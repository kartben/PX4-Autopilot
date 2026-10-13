/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file px4_romfs.c
 *
 * Read-only file system of the files built into the firmware, mounted at
 * /etc, as the ROMFS of NuttX: the component metadata is in /etc/extras, where
 * MAVLink FTP serves it to the ground station and the logger writes the events
 * to the logs. The files are read in place from the image.
 */

#include <px4_romfs.h>

#include <errno.h>
#include <string.h>

#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(px4_romfs, CONFIG_LOG_DEFAULT_LEVEL);

#define PX4_ROMFS_TYPE FS_TYPE_EXTERNAL_BASE
#define PX4_ROMFS_BLOCK_SIZE 512

/* an open file and its position, or an open directory and the index of its next entry */
struct px4_romfs_open {
	const struct px4_romfs_entry *entry;
	size_t position;
};

K_MEM_SLAB_DEFINE_STATIC_TYPE(px4_romfs_open_slab, struct px4_romfs_open,
			      CONFIG_ZVFS_OPEN_ADD_SIZE_PX4_COMPONENT_METADATA);

static const struct px4_romfs_entry px4_romfs_root = {
	.path = "",
};

static bool px4_romfs_is_dir(const struct px4_romfs_entry *entry)
{
	return entry->data == NULL;
}

/* the entry of an absolute path, which starts with the mount point */
static const struct px4_romfs_entry *px4_romfs_find(const struct fs_mount_t *mp, const char *path)
{
	path += mp->mountp_len;

	while (*path == '/') {
		path++;
	}

	size_t length = strlen(path);

	while ((length > 0) && (path[length - 1] == '/')) {
		length--;
	}

	if (length == 0) {
		return &px4_romfs_root;
	}

	for (size_t i = 0; i < px4_romfs_entry_count; i++) {
		const struct px4_romfs_entry *entry = &px4_romfs_entries[i];

		if ((strncmp(entry->path, path, length) == 0) && (entry->path[length] == '\0')) {
			return entry;
		}
	}

	return NULL;
}

/* true if the entry is in the directory, and not in one of its subdirectories */
static bool px4_romfs_in_dir(const struct px4_romfs_entry *dir, const struct px4_romfs_entry *entry)
{
	const size_t length = strlen(dir->path);
	const char *name = entry->path;

	if (length > 0) {
		if ((strncmp(name, dir->path, length) != 0) || (name[length] != '/')) {
			return false;
		}

		name += length + 1;
	}

	return (name[0] != '\0') && (strchr(name, '/') == NULL);
}

static void px4_romfs_dirent(const struct px4_romfs_entry *entry, struct fs_dirent *dirent)
{
	const char *slash = strrchr(entry->path, '/');
	const char *name = (slash != NULL) ? (slash + 1) : entry->path;
	const size_t length = MIN(strlen(name), sizeof(dirent->name) - 1);

	dirent->type = px4_romfs_is_dir(entry) ? FS_DIR_ENTRY_DIR : FS_DIR_ENTRY_FILE;
	memcpy(dirent->name, name, length);
	dirent->name[length] = '\0';
	dirent->size = entry->size;
}

static int px4_romfs_alloc(const struct px4_romfs_entry *entry, void **open)
{
	struct px4_romfs_open *open_entry;

	if (k_mem_slab_alloc(&px4_romfs_open_slab, (void **)&open_entry, K_NO_WAIT) != 0) {
		return -ENOMEM;
	}

	open_entry->entry = entry;
	open_entry->position = 0;
	*open = open_entry;
	return 0;
}

static int px4_romfs_open(struct fs_file_t *filp, const char *path, fs_mode_t flags)
{
	ARG_UNUSED(flags);

	const struct px4_romfs_entry *entry = px4_romfs_find(filp->mp, path);

	if (entry == NULL) {
		return -ENOENT;
	}

	if (px4_romfs_is_dir(entry)) {
		return -EISDIR;
	}

	return px4_romfs_alloc(entry, &filp->filep);
}

static ssize_t px4_romfs_read(struct fs_file_t *filp, void *dest, size_t nbytes)
{
	struct px4_romfs_open *file = filp->filep;
	const size_t length = MIN(nbytes, file->entry->size - file->position);

	memcpy(dest, &file->entry->data[file->position], length);
	file->position += length;
	return length;
}

static int px4_romfs_lseek(struct fs_file_t *filp, off_t off, int whence)
{
	struct px4_romfs_open *file = filp->filep;
	off_t position;

	switch (whence) {
	case FS_SEEK_SET:
		position = off;
		break;

	case FS_SEEK_CUR:
		position = (off_t)file->position + off;
		break;

	case FS_SEEK_END:
		position = (off_t)file->entry->size + off;
		break;

	default:
		return -EINVAL;
	}

	/* within the file, as with FAT */
	if ((position < 0) || (position > (off_t)file->entry->size)) {
		return -EINVAL;
	}

	file->position = position;
	return 0;
}

static off_t px4_romfs_tell(struct fs_file_t *filp)
{
	const struct px4_romfs_open *file = filp->filep;

	return file->position;
}

static int px4_romfs_close(struct fs_file_t *filp)
{
	k_mem_slab_free(&px4_romfs_open_slab, filp->filep);
	return 0;
}

static int px4_romfs_opendir(struct fs_dir_t *dirp, const char *path)
{
	const struct px4_romfs_entry *entry = px4_romfs_find(dirp->mp, path);

	if (entry == NULL) {
		return -ENOENT;
	}

	if (!px4_romfs_is_dir(entry)) {
		return -ENOTDIR;
	}

	return px4_romfs_alloc(entry, &dirp->dirp);
}

static int px4_romfs_readdir(struct fs_dir_t *dirp, struct fs_dirent *dirent)
{
	struct px4_romfs_open *dir = dirp->dirp;

	while (dir->position < px4_romfs_entry_count) {
		const struct px4_romfs_entry *entry = &px4_romfs_entries[dir->position++];

		if (px4_romfs_in_dir(dir->entry, entry)) {
			px4_romfs_dirent(entry, dirent);
			return 0;
		}
	}

	/* end of the directory */
	dirent->name[0] = '\0';
	return 0;
}

static int px4_romfs_closedir(struct fs_dir_t *dirp)
{
	k_mem_slab_free(&px4_romfs_open_slab, dirp->dirp);
	return 0;
}

static int px4_romfs_mount(struct fs_mount_t *mountp)
{
	ARG_UNUSED(mountp);
	return 0;
}

static int px4_romfs_unmount(struct fs_mount_t *mountp)
{
	ARG_UNUSED(mountp);
	return 0;
}

static int px4_romfs_stat(struct fs_mount_t *mountp, const char *path, struct fs_dirent *dirent)
{
	const struct px4_romfs_entry *entry = px4_romfs_find(mountp, path);

	if (entry == NULL) {
		return -ENOENT;
	}

	px4_romfs_dirent(entry, dirent);
	return 0;
}

/* stat() also gets the block size of the file system */
static int px4_romfs_statvfs(struct fs_mount_t *mountp, const char *path, struct fs_statvfs *stat)
{
	ARG_UNUSED(mountp);
	ARG_UNUSED(path);

	size_t size = 0;

	for (size_t i = 0; i < px4_romfs_entry_count; i++) {
		size += px4_romfs_entries[i].size;
	}

	stat->f_bsize = PX4_ROMFS_BLOCK_SIZE;
	stat->f_frsize = PX4_ROMFS_BLOCK_SIZE;
	stat->f_blocks = DIV_ROUND_UP(size, PX4_ROMFS_BLOCK_SIZE);
	stat->f_bfree = 0;
	return 0;
}

static const struct fs_file_system_t px4_romfs_fs = {
	.open = px4_romfs_open,
	.read = px4_romfs_read,
	.lseek = px4_romfs_lseek,
	.tell = px4_romfs_tell,
	.close = px4_romfs_close,
	.opendir = px4_romfs_opendir,
	.readdir = px4_romfs_readdir,
	.closedir = px4_romfs_closedir,
	.mount = px4_romfs_mount,
	.unmount = px4_romfs_unmount,
	.stat = px4_romfs_stat,
	.statvfs = px4_romfs_statvfs,
};

/*
 * The file system core refuses to write, create, delete or rename files on a read-only mount, and
 * to mount two file systems with the same data at mount points of the same length, such as /SD:.
 */
static struct fs_mount_t px4_romfs_mnt = {
	.type = PX4_ROMFS_TYPE,
	.mnt_point = "/etc",
	.fs_data = (void *)px4_romfs_entries,
	.flags = FS_MOUNT_FLAG_READ_ONLY,
};

static int px4_romfs_init(void)
{
	int ret = fs_register(PX4_ROMFS_TYPE, &px4_romfs_fs);

	if (ret == 0) {
		ret = fs_mount(&px4_romfs_mnt);
	}

	if (ret != 0) {
		LOG_ERR("mount at %s failed (%d)", px4_romfs_mnt.mnt_point, ret);
	}

	return ret;
}

SYS_INIT(px4_romfs_init, POST_KERNEL, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
