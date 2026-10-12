/* HH_261005 - Count every entry through no-follow directory descriptors without payload reads or cached membership. */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

typedef struct {
    uint64_t bytes, entries, directories, regular, special;
} quota_counts;

static int same_directory(const struct stat *a, const struct stat *b) {
    return S_ISDIR(b->st_mode) && a->st_dev == b->st_dev && a->st_ino == b->st_ino;
}

static int add_value(uint64_t *target, uint64_t amount) {
    if (UINT64_MAX - *target < amount) return EOVERFLOW;
    *target += amount;
    return 0;
}

static int walk_owned(int fd, const struct stat *expected, quota_counts *out, unsigned depth) {
    /* HH_261005 - fd ownership transfers here even on failure; cap recursion and reject directory replacement. */
    struct stat current;
    if (depth > 512) { close(fd); return ELOOP; }
    if (fstat(fd, &current)) { int err = errno; close(fd); return err; }
    if (!same_directory(expected, &current)) { close(fd); return ESTALE; }
    DIR *stream = fdopendir(fd);
    if (!stream) { int err = errno; close(fd); return err; }
    int err = add_value(&out->directories, 1);
    struct dirent *entry;
    while (!err) {
        errno = 0;
        entry = readdir(stream);
        if (!entry) { err = errno; break; }
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        if ((err = add_value(&out->entries, 1))) break;
        struct stat metadata;
        if (fstatat(dirfd(stream), entry->d_name, &metadata, AT_SYMLINK_NOFOLLOW)) { err = errno; break; }
        if (S_ISLNK(metadata.st_mode)) { err = ELOOP; break; }
        if (S_ISDIR(metadata.st_mode)) {
            int child = openat(dirfd(stream), entry->d_name, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
            if (child < 0) { err = errno; break; }
            err = walk_owned(child, &metadata, out, depth + 1);
        } else if (S_ISREG(metadata.st_mode)) {
            if (metadata.st_size < 0) { err = EOVERFLOW; break; }
            if (!(err = add_value(&out->bytes, (uint64_t)metadata.st_size))) err = add_value(&out->regular, 1);
        } else {
            err = add_value(&out->special, 1);
        }
    }
    /* HH_261005 - An unstable directory traversal fails closed instead of reporting a partial safe quota. */
    if (!err && fstat(dirfd(stream), &current)) err = errno;
    if (!err && (!same_directory(expected, &current)
        || expected->st_mtim.tv_sec != current.st_mtim.tv_sec || expected->st_mtim.tv_nsec != current.st_mtim.tv_nsec
        || expected->st_ctim.tv_sec != current.st_ctim.tv_sec || expected->st_ctim.tv_nsec != current.st_ctim.tv_nsec)) err = ESTALE;
    if (closedir(stream) && !err) err = errno;
    return err;
}

int hh_full_tree_quota_v1(const char *path, quota_counts *out) {
    /* HH_261005 - Report no valid count on error; the caller preserves failed invocation receipts. */
    if (!path || !out) return -EINVAL;
    memset(out, 0, sizeof(*out));
    struct stat root;
    if (lstat(path, &root)) return errno == ENOENT ? 0 : -errno;
    if (!S_ISDIR(root.st_mode) || S_ISLNK(root.st_mode)) return -ENOTDIR;
    int fd = open(path, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (fd < 0) return -errno;
    int err = walk_owned(fd, &root, out, 0);
    return -err;
}
