/*
 * AF_XDP (XSK) receive utility for AVTP listeners.
 * Raw kernel syscall implementation — no libbpf/libxdp dependency.
 * Uses only: linux/if_xdp.h, linux/bpf.h, standard POSIX.
 */

#include "xsk-util.h"

#include <errno.h>
#include <linux/bpf.h>
#include <linux/ethtool.h>
#include <linux/if_link.h>
#include <linux/if_xdp.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

/* BPF instruction opcodes (not exported to userspace) */
#define BPF_OP_LDX_MEM_W  (0x61)
#define BPF_OP_MOV64_IMM   (0xb7)
#define BPF_OP_CALL        (0x85)
#define BPF_OP_EXIT        (0x95)
#define BPF_OP_LD_DW_IMM   (0x18)

#define BPF_HELPER_redirect_map  51

struct xsk_ring {
	uint32_t *producer;
	uint32_t *consumer;
	uint32_t *flags;
	uint32_t mask;
	uint32_t size;
	void *ring;		/* desc array base */
	uint32_t cached_prod;
	uint32_t cached_cons;
};

struct xsk_ctx {
	int xsk_fd;
	int map_fd;
	int prog_fd;
	int nl_fd;		/* netlink socket for XDP attach/detach */
	int ifindex;
	uint8_t *umem_area;
	uint32_t umem_size;
	uint32_t frame_size;
	uint32_t num_frames;
	struct xsk_ring fq;	/* fill ring (producer) */
	struct xsk_ring rx;	/* rx ring (consumer) */
	void *fq_map;
	size_t fq_map_sz;
	void *rx_map;
	size_t rx_map_sz;
};

static inline long sys_bpf(enum bpf_cmd cmd, union bpf_attr *attr,
			    unsigned int size)
{
	return syscall(__NR_bpf, cmd, attr, size);
}

static int create_xskmap(uint32_t max_entries)
{
	union bpf_attr attr;

	memset(&attr, 0, sizeof(attr));
	attr.map_type = BPF_MAP_TYPE_XSKMAP;
	attr.key_size = sizeof(uint32_t);
	attr.value_size = sizeof(int);
	attr.max_entries = max_entries;
	strncpy(attr.map_name, "xsk_map", sizeof(attr.map_name));

	return sys_bpf(BPF_MAP_CREATE, &attr, sizeof(attr));
}

static int load_xdp_prog(int map_fd)
{
	/*
	 * XDP program equivalent to:
	 *   return bpf_redirect_map(&xsk_map, ctx->rx_queue_index, XDP_PASS);
	 *
	 * Instructions:
	 *   0: r2 = *(u32 *)(r1 + 16)   // rx_queue_index from xdp_md
	 *   1-2: r1 = map_fd             // LD_DW_IMM (pseudo map fd, 2 insns)
	 *   3: r3 = 2                    // XDP_PASS fallback
	 *   4: call bpf_redirect_map
	 *   5: exit
	 */
#define I(c, d, s, o, i) \
	{.code = (c), .dst_reg = (d), .src_reg = (s), .off = (o), .imm = (i)}
	struct bpf_insn prog[] = {
		I(BPF_OP_LDX_MEM_W, 2, 1, 16, 0),
		I(BPF_OP_LD_DW_IMM, 1, BPF_PSEUDO_MAP_FD, 0, map_fd),
		I(0, 0, 0, 0, 0),
		I(BPF_OP_MOV64_IMM, 3, 0, 0, 2),
		I(BPF_OP_CALL, 0, 0, 0, BPF_HELPER_redirect_map),
		I(BPF_OP_EXIT, 0, 0, 0, 0),
	};
#undef I
	char license[] = "GPL";
	char log_buf[4096] = {0};
	union bpf_attr attr;

	memset(&attr, 0, sizeof(attr));
	attr.prog_type = BPF_PROG_TYPE_XDP;
	attr.insn_cnt = sizeof(prog) / sizeof(prog[0]);
	attr.insns = (uintptr_t)prog;
	attr.license = (uintptr_t)license;
	attr.log_level = 1;
	attr.log_size = sizeof(log_buf);
	attr.log_buf = (uintptr_t)log_buf;
	strncpy(attr.prog_name, "xsk_redir", sizeof(attr.prog_name));

	int fd = sys_bpf(BPF_PROG_LOAD, &attr, sizeof(attr));
	if (fd < 0)
		fprintf(stderr, "xsk: BPF_PROG_LOAD failed: %s\n%s\n",
			strerror(errno), log_buf);
	return fd;
}

static int attach_xdp(int prog_fd, int ifindex)
{
	struct {
		struct nlmsghdr nh;
		struct ifinfomsg ifi;
		char buf[256];
	} req;
	struct nlattr *xdp_attr, *nla;
	int nl_fd, ret;

	nl_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (nl_fd < 0) {
		perror("xsk: netlink socket");
		return -1;
	}

	struct sockaddr_nl sa = {.nl_family = AF_NETLINK};
	if (bind(nl_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		perror("xsk: netlink bind");
		close(nl_fd);
		return -1;
	}

	memset(&req, 0, sizeof(req));
	req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(req.ifi));
	req.nh.nlmsg_type = RTM_SETLINK;
	req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
	req.ifi.ifi_family = AF_UNSPEC;
	req.ifi.ifi_index = ifindex;

	/* Nested IFLA_XDP { IFLA_XDP_FD, IFLA_XDP_FLAGS } */
	xdp_attr = (struct nlattr *)((char *)&req + req.nh.nlmsg_len);
	xdp_attr->nla_type = IFLA_XDP | NLA_F_NESTED;

	/* IFLA_XDP_FD */
	nla = (struct nlattr *)((char *)xdp_attr + NLA_HDRLEN);
	nla->nla_type = IFLA_XDP_FD;
	nla->nla_len = NLA_HDRLEN + sizeof(int);
	memcpy((char *)nla + NLA_HDRLEN, &prog_fd, sizeof(int));

	/* IFLA_XDP_FLAGS */
	nla = (struct nlattr *)((char *)nla + NLA_ALIGN(nla->nla_len));
	nla->nla_type = IFLA_XDP_FLAGS;
	nla->nla_len = NLA_HDRLEN + sizeof(uint32_t);
	uint32_t flags = XDP_FLAGS_DRV_MODE;
	memcpy((char *)nla + NLA_HDRLEN, &flags, sizeof(flags));

	xdp_attr->nla_len = (char *)nla + NLA_ALIGN(nla->nla_len) -
			     (char *)xdp_attr;
	req.nh.nlmsg_len = (char *)nla + NLA_ALIGN(nla->nla_len) - (char *)&req;

	ret = send(nl_fd, &req, req.nh.nlmsg_len, 0);
	if (ret < 0) {
		perror("xsk: netlink send");
		close(nl_fd);
		return -1;
	}

	/* Read ACK */
	char resp[4096];
	ret = recv(nl_fd, resp, sizeof(resp), 0);
	if (ret < 0) {
		perror("xsk: netlink recv");
		close(nl_fd);
		return -1;
	}
	struct nlmsghdr *rnh = (struct nlmsghdr *)resp;
	if (rnh->nlmsg_type == NLMSG_ERROR) {
		struct nlmsgerr *err = NLMSG_DATA(rnh);
		if (err->error) {
			fprintf(stderr, "xsk: XDP attach failed: %s (err=%d)\n",
				strerror(-err->error), err->error);
			close(nl_fd);
			return -1;
		}
	} else {
		fprintf(stderr, "xsk: netlink unexpected msg type %d\n",
			rnh->nlmsg_type);
		close(nl_fd);
		return -1;
	}

	/* Keep nl_fd open — closing it doesn't detach, but we return it
	 * so destroy can detach by sending fd=-1 */
	return nl_fd;
}

static int xskmap_update(int map_fd, uint32_t key, int xsk_fd)
{
	union bpf_attr attr;

	memset(&attr, 0, sizeof(attr));
	attr.map_fd = map_fd;
	attr.key = (uintptr_t)&key;
	attr.value = (uintptr_t)&xsk_fd;
	attr.flags = 0;

	return sys_bpf(BPF_MAP_UPDATE_ELEM, &attr, sizeof(attr));
}

static int setup_ring(int fd, struct xsk_ring *ring,
		      struct xdp_ring_offset *off,
		      uint32_t size, off_t pgoff,
		      void **map_out, size_t *map_sz_out)
{
	size_t mmap_sz;
	void *map;

	mmap_sz = off->desc + size * sizeof(struct xdp_desc);
	/* For fill ring, descs are uint64_t addrs, not xdp_desc */
	if (pgoff == XDP_UMEM_PGOFF_FILL_RING)
		mmap_sz = off->desc + size * sizeof(uint64_t);

	map = mmap(NULL, mmap_sz, PROT_READ | PROT_WRITE,
		   MAP_SHARED | MAP_POPULATE, fd, pgoff);
	if (map == MAP_FAILED)
		return -1;

	ring->producer = (uint32_t *)((uint8_t *)map + off->producer);
	ring->consumer = (uint32_t *)((uint8_t *)map + off->consumer);
	ring->flags = (uint32_t *)((uint8_t *)map + off->flags);
	ring->ring = (uint8_t *)map + off->desc;
	ring->size = size;
	ring->mask = size - 1;
	ring->cached_prod = 0;
	ring->cached_cons = 0;

	*map_out = map;
	*map_sz_out = mmap_sz;
	return 0;
}

/* Ring helpers with memory ordering */
static inline uint32_t ring_prod_load(const struct xsk_ring *r)
{
	return __atomic_load_n(r->producer, __ATOMIC_ACQUIRE);
}

static inline void ring_prod_store(struct xsk_ring *r, uint32_t val)
{
	__atomic_store_n(r->producer, val, __ATOMIC_RELEASE);
}

static inline uint32_t ring_cons_load(const struct xsk_ring *r)
{
	return __atomic_load_n(r->consumer, __ATOMIC_ACQUIRE);
}

static inline void ring_cons_store(struct xsk_ring *r, uint32_t val)
{
	__atomic_store_n(r->consumer, val, __ATOMIC_RELEASE);
}

struct xsk_ctx *xsk_ctx_create(const struct xsk_config *cfg)
{
	struct xsk_ctx *ctx;
	struct xdp_mmap_offsets offsets;
	struct xdp_umem_reg umem_reg;
	struct sockaddr_xdp sxdp;
	socklen_t optlen;
	uint32_t ring_size;
	int ifindex;
	int ret;

	ifindex = if_nametoindex(cfg->ifname);
	if (!ifindex) {
		fprintf(stderr, "xsk: unknown interface %s\n", cfg->ifname);
		return NULL;
	}

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		return NULL;

	ctx->xsk_fd = -1;
	ctx->map_fd = -1;
	ctx->prog_fd = -1;
	ctx->nl_fd = -1;
	ctx->ifindex = ifindex;
	ctx->frame_size = cfg->frame_size;
	ctx->num_frames = cfg->num_frames;
	ctx->umem_size = cfg->frame_size * cfg->num_frames;

	/* Allocate UMEM */
	ctx->umem_area = mmap(NULL, ctx->umem_size, PROT_READ | PROT_WRITE,
			      MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE,
			      -1, 0);
	if (ctx->umem_area == MAP_FAILED) {
		perror("xsk: mmap UMEM");
		goto err;
	}

	/* BPF: create XSKMAP, load prog, attach */
	ctx->map_fd = create_xskmap(cfg->queue_id + 1);
	if (ctx->map_fd < 0)
		goto err;

	ctx->prog_fd = load_xdp_prog(ctx->map_fd);
	if (ctx->prog_fd < 0)
		goto err;

	ctx->nl_fd = attach_xdp(ctx->prog_fd, ifindex);
	if (ctx->nl_fd < 0)
		goto err;

	/* Create AF_XDP socket */
	ctx->xsk_fd = socket(AF_XDP, SOCK_RAW, 0);
	if (ctx->xsk_fd < 0) {
		perror("xsk: socket(AF_XDP)");
		goto err;
	}

	/* Register UMEM */
	memset(&umem_reg, 0, sizeof(umem_reg));
	umem_reg.addr = (uintptr_t)ctx->umem_area;
	umem_reg.len = ctx->umem_size;
	umem_reg.chunk_size = cfg->frame_size;
	umem_reg.headroom = 0;

	ret = setsockopt(ctx->xsk_fd, SOL_XDP, XDP_UMEM_REG,
			 &umem_reg, sizeof(umem_reg));
	if (ret < 0) {
		perror("xsk: XDP_UMEM_REG");
		goto err;
	}

	/* Configure ring sizes */
	ring_size = cfg->num_frames;
	ret = setsockopt(ctx->xsk_fd, SOL_XDP, XDP_UMEM_FILL_RING,
			 &ring_size, sizeof(ring_size));
	if (ret < 0) {
		perror("xsk: XDP_UMEM_FILL_RING");
		goto err;
	}

	ret = setsockopt(ctx->xsk_fd, SOL_XDP, XDP_UMEM_COMPLETION_RING,
			 &ring_size, sizeof(ring_size));
	if (ret < 0) {
		perror("xsk: XDP_UMEM_COMPLETION_RING");
		goto err;
	}

	ret = setsockopt(ctx->xsk_fd, SOL_XDP, XDP_RX_RING,
			 &ring_size, sizeof(ring_size));
	if (ret < 0) {
		perror("xsk: XDP_RX_RING");
		goto err;
	}

	/* Get mmap offsets */
	optlen = sizeof(offsets);
	ret = getsockopt(ctx->xsk_fd, SOL_XDP, XDP_MMAP_OFFSETS,
			 &offsets, &optlen);
	if (ret < 0) {
		perror("xsk: XDP_MMAP_OFFSETS");
		goto err;
	}

	/* Map fill ring */
	ret = setup_ring(ctx->xsk_fd, &ctx->fq, &offsets.fr,
			 ring_size, XDP_UMEM_PGOFF_FILL_RING,
			 &ctx->fq_map, &ctx->fq_map_sz);
	if (ret < 0) {
		perror("xsk: mmap fill ring");
		goto err;
	}

	/* Map RX ring */
	ret = setup_ring(ctx->xsk_fd, &ctx->rx, &offsets.rx,
			 ring_size, XDP_PGOFF_RX_RING,
			 &ctx->rx_map, &ctx->rx_map_sz);
	if (ret < 0) {
		perror("xsk: mmap rx ring");
		goto err;
	}

	/* Pre-populate fill ring */
	uint64_t *fq_descs = (uint64_t *)ctx->fq.ring;
	for (uint32_t i = 0; i < cfg->num_frames; i++)
		fq_descs[i & ctx->fq.mask] = (uint64_t)i * cfg->frame_size;
	ring_prod_store(&ctx->fq, cfg->num_frames);
	ctx->fq.cached_prod = cfg->num_frames;

	/* Bind socket — try zero-copy, then copy, then no flags */
	memset(&sxdp, 0, sizeof(sxdp));
	sxdp.sxdp_family = AF_XDP;
	sxdp.sxdp_ifindex = ifindex;
	sxdp.sxdp_queue_id = cfg->queue_id;

	if (cfg->zero_copy) {
		sxdp.sxdp_flags = XDP_ZEROCOPY | XDP_USE_NEED_WAKEUP;
		ret = bind(ctx->xsk_fd, (struct sockaddr *)&sxdp, sizeof(sxdp));
		if (ret == 0)
			goto bind_ok;
	}

	sxdp.sxdp_flags = XDP_COPY | XDP_USE_NEED_WAKEUP;
	ret = bind(ctx->xsk_fd, (struct sockaddr *)&sxdp, sizeof(sxdp));
	if (ret == 0)
		goto bind_ok;

	sxdp.sxdp_flags = 0;
	ret = bind(ctx->xsk_fd, (struct sockaddr *)&sxdp, sizeof(sxdp));
	if (ret == 0)
		goto bind_ok;

	fprintf(stderr, "xsk: bind failed: %s (queue=%u ifindex=%d)\n",
		strerror(errno), sxdp.sxdp_queue_id, sxdp.sxdp_ifindex);
	goto err;

bind_ok:

	/* Insert socket into XSKMAP */
	ret = xskmap_update(ctx->map_fd, cfg->queue_id, ctx->xsk_fd);
	if (ret < 0) {
		fprintf(stderr, "xsk: map update failed: %s\n",
			strerror(errno));
		goto err;
	}

	/* Report mode */
	struct xdp_options opts = {0};
	optlen = sizeof(opts);
	if (getsockopt(ctx->xsk_fd, SOL_XDP, XDP_OPTIONS,
		       &opts, &optlen) == 0) {
		fprintf(stderr, "xsk: %s mode on %s queue %u (%u x %u frames)\n",
			(opts.flags & XDP_OPTIONS_ZEROCOPY) ?
			"zero-copy" : "copy",
			cfg->ifname, cfg->queue_id,
			cfg->num_frames, cfg->frame_size);
	}

	return ctx;

err:
	xsk_ctx_destroy(ctx);
	return NULL;
}

static void detach_xdp(int nl_fd, int ifindex)
{
	struct {
		struct nlmsghdr nh;
		struct ifinfomsg ifi;
		char buf[256];
	} req;
	struct nlattr *xdp_attr, *nla;

	memset(&req, 0, sizeof(req));
	req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(req.ifi));
	req.nh.nlmsg_type = RTM_SETLINK;
	req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
	req.ifi.ifi_family = AF_UNSPEC;
	req.ifi.ifi_index = ifindex;

	xdp_attr = (struct nlattr *)((char *)&req + req.nh.nlmsg_len);
	xdp_attr->nla_type = IFLA_XDP | NLA_F_NESTED;

	nla = (struct nlattr *)((char *)xdp_attr + NLA_HDRLEN);
	nla->nla_type = IFLA_XDP_FD;
	nla->nla_len = NLA_HDRLEN + sizeof(int);
	int detach_fd = -1;
	memcpy((char *)nla + NLA_HDRLEN, &detach_fd, sizeof(int));

	xdp_attr->nla_len = (char *)nla + NLA_ALIGN(nla->nla_len) -
			     (char *)xdp_attr;
	req.nh.nlmsg_len = (char *)nla + NLA_ALIGN(nla->nla_len) - (char *)&req;

	send(nl_fd, &req, req.nh.nlmsg_len, 0);
}

void xsk_ctx_destroy(struct xsk_ctx *ctx)
{
	if (!ctx)
		return;

	/* Order matters for zero-copy: stop redirects first, then close
	 * socket (which unhooks UMEM from driver), then unmap. */
	if (ctx->nl_fd >= 0) {
		detach_xdp(ctx->nl_fd, ctx->ifindex);
		close(ctx->nl_fd);
		ctx->nl_fd = -1;
	}
	if (ctx->prog_fd >= 0) {
		close(ctx->prog_fd);
		ctx->prog_fd = -1;
	}
	if (ctx->map_fd >= 0) {
		close(ctx->map_fd);
		ctx->map_fd = -1;
	}
	if (ctx->xsk_fd >= 0) {
		close(ctx->xsk_fd);
		ctx->xsk_fd = -1;
	}
	if (ctx->fq_map)
		munmap(ctx->fq_map, ctx->fq_map_sz);
	if (ctx->rx_map)
		munmap(ctx->rx_map, ctx->rx_map_sz);
	if (ctx->umem_area && ctx->umem_area != MAP_FAILED)
		munmap(ctx->umem_area, ctx->umem_size);
	free(ctx);
}

int xsk_ctx_fd(const struct xsk_ctx *ctx)
{
	return ctx->xsk_fd;
}

uint32_t xsk_ctx_rx_peek(struct xsk_ctx *ctx, uint32_t max_batch)
{
	uint32_t prod = ring_prod_load(&ctx->rx);
	uint32_t avail = prod - ctx->rx.cached_cons;

	if (avail > max_batch)
		avail = max_batch;
	return avail;
}

const uint8_t *xsk_ctx_rx_packet(struct xsk_ctx *ctx, uint32_t idx,
				 uint32_t *len)
{
	uint32_t ring_idx = (ctx->rx.cached_cons + idx) & ctx->rx.mask;
	struct xdp_desc *desc = &((struct xdp_desc *)ctx->rx.ring)[ring_idx];

	*len = desc->len;
	return ctx->umem_area + desc->addr;
}

void xsk_ctx_rx_release(struct xsk_ctx *ctx, uint32_t count)
{
	/* Recycle consumed frames back to fill ring */
	uint64_t *fq_descs = (uint64_t *)ctx->fq.ring;
	uint32_t fq_prod = ctx->fq.cached_prod;

	for (uint32_t i = 0; i < count; i++) {
		uint32_t rx_idx = (ctx->rx.cached_cons + i) & ctx->rx.mask;
		struct xdp_desc *desc =
			&((struct xdp_desc *)ctx->rx.ring)[rx_idx];
		fq_descs[fq_prod & ctx->fq.mask] = desc->addr;
		fq_prod++;
	}
	ring_prod_store(&ctx->fq, fq_prod);
	ctx->fq.cached_prod = fq_prod;

	/* Advance RX consumer */
	ctx->rx.cached_cons += count;
	ring_cons_store(&ctx->rx, ctx->rx.cached_cons);
}

bool xsk_ctx_needs_wakeup(const struct xsk_ctx *ctx)
{
	return *ctx->fq.flags & XDP_RING_NEED_WAKEUP;
}

int xsk_configure_flow_steering(const char *ifname, const uint8_t *dst_mac,
				uint32_t queue_id)
{
	int fd, ret;
	struct ifreq ifr;

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
		return -1;

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

	/* Set combined queue count to at least queue_id + 1 */
	struct ethtool_channels ch = {
		.cmd = ETHTOOL_GCHANNELS,
	};
	ifr.ifr_data = (void *)&ch;
	ret = ioctl(fd, SIOCETHTOOL, &ifr);
	if (ret < 0) {
		perror("xsk: ETHTOOL_GCHANNELS");
		close(fd);
		return -1;
	}

	if (ch.combined_count < queue_id + 1) {
		ch.cmd = ETHTOOL_SCHANNELS;
		ch.combined_count = queue_id + 1;
		ret = ioctl(fd, SIOCETHTOOL, &ifr);
		if (ret < 0) {
			perror("xsk: ETHTOOL_SCHANNELS");
			close(fd);
			return -1;
		}
	}

	/* Insert flow steering rule: dst MAC → queue */
	struct ethtool_rxnfc nfc;

	memset(&nfc, 0, sizeof(nfc));
	nfc.cmd = ETHTOOL_SRXCLSRLINS;
	nfc.fs.flow_type = ETHER_FLOW;
	memcpy(nfc.fs.h_u.ether_spec.h_dest, dst_mac, 6);
	memset(nfc.fs.m_u.ether_spec.h_dest, 0xff, 6);
	nfc.fs.ring_cookie = queue_id;
	nfc.fs.location = RX_CLS_LOC_ANY;

	ifr.ifr_data = (void *)&nfc;
	ret = ioctl(fd, SIOCETHTOOL, &ifr);
	if (ret < 0 && errno != EEXIST) {
		perror("xsk: ETHTOOL_SRXCLSRLINS");
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}
