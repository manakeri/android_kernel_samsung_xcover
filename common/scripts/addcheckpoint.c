/*
 * Copyright (C) 2010 Marvell Inc.
 * Created by Lei Wen <leiwen@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * addcheckpoint.c
 *
 * Append the checkpoint to the yaffs image to avoid the first mount backwards scanning.
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#define CONFIG_YAFFS_UTIL
#define CONFIG_YAFFS_YAFFS2
#include "../fs/yaffs2/yaffs_ecc.h"
#include "../fs/yaffs2/yaffs_guts.h"

#include "../fs/yaffs2/yaffs_tagsvalidity.h"
#include "../fs/yaffs2/yaffs_packedtags2.h"

#define BASE_ID	(0x101)
#define MAX_SPARE_SIZE (256)
#define TMP_FILE "/tmp/addyaffscheckpoint_tmp"
static unsigned int blocksize_shift;
static unsigned int pageNum;
static unsigned int chunkSize;
static unsigned int spareSize;

struct objInfo {
	unsigned int chunks;
	unsigned int parentId;
	unsigned int variantType;
	unsigned int startChunk;
	unsigned int filesize;
};

static int write_chunk(int fd, unsigned char *data, unsigned int objId, unsigned int chunkId, unsigned int nBytes)
{
	yaffs_PackedTags2 pt;
	char spare[MAX_SPARE_SIZE];
	int error;

	error = write(fd,data,chunkSize);
	if(error < 0) return error;

	memset(&pt, 0, sizeof(pt));
	pt.t.sequenceNumber = YAFFS_SEQUENCE_CHECKPOINT_DATA;
	pt.t.objectId = objId;
	pt.t.chunkId = chunkId;
	pt.t.byteCount = nBytes;
	memcpy(spare, &pt, sizeof(pt));
	memset(spare + sizeof(pt), 0xff, spareSize - sizeof(pt));

	return write(fd, spare, spareSize);
}

static void yaffs_WriteCheckpointValidityMarker(int tmp_fd, int head)
{
	yaffs_CheckpointValidity cp;

	memset(&cp, 0, sizeof(cp));
	cp.structType = sizeof(cp);
	cp.magic = YAFFS_MAGIC;
	cp.version = YAFFS_CHECKPOINT_VERSION;
	cp.head = (head) ? 1 : 0;

	write(tmp_fd, &cp, sizeof(cp));
}

static int yaffs_WriteCheckpointDevice(int tmp_fd, unsigned long part_size, unsigned long file_size,
				       int objnum, struct objInfo *objInfo)
{
	yaffs_CheckpointDevice cp;
	yaffs_BlockInfo *blockInfo;
	int part_blocks, page_alloced, i, pages, chunkBitmapStride, ret, page_sum;
	char *bitmap;

	chunkBitmapStride = (pageNum + 7) / 8;
	assert(chunkBitmapStride <= sizeof(yaffs_BlockInfo));

	part_blocks = part_size >> blocksize_shift;
	file_size = (file_size / (spareSize + chunkSize)) * chunkSize;
	memset(&cp, 0, sizeof(cp));
	cp.structType = sizeof(cp);
	part_size = part_size - file_size;
	cp.nErasedBlocks = part_size >> blocksize_shift;
	cp.allocationBlock = part_blocks - cp.nErasedBlocks;
	cp.nFreeChunks = part_size / chunkSize;
	page_alloced = (cp.nFreeChunks / pageNum) * pageNum;
	page_alloced = cp.nFreeChunks - page_alloced;
	cp.allocationPage = (!page_alloced) ? 0 : (pageNum - page_alloced);
	cp.sequenceNumber = YAFFS_LOWEST_SEQUENCE_NUMBER;

	write(tmp_fd, &cp, sizeof(cp));
	/* Write block info */
	blockInfo = malloc(sizeof(yaffs_BlockInfo) * part_blocks);
	if (!blockInfo) {
		printf("malloc block info fail\n");
		return -1;

	}

	for (i = 0; i < cp.allocationBlock - 1; i ++) {
		memset(&blockInfo[i], 0, sizeof(yaffs_BlockInfo));
		blockInfo[i].blockState = YAFFS_BLOCK_STATE_FULL;
		blockInfo[i].pagesInUse = pageNum;
		blockInfo[i].sequenceNumber = YAFFS_LOWEST_SEQUENCE_NUMBER;
	}

	if (cp.allocationPage != 0) {
		memset(&blockInfo[i], 0, sizeof(yaffs_BlockInfo));
		blockInfo[i].blockState = YAFFS_BLOCK_STATE_ALLOCATING;
		blockInfo[i].sequenceNumber = YAFFS_LOWEST_SEQUENCE_NUMBER;
		blockInfo[i].pagesInUse = cp.allocationPage;
		blockInfo[i].hasShrinkHeader = 1;
		i ++;
	}

	ret = i;
	for (; i < part_blocks; i ++) {
		memset(&blockInfo[i], 0, sizeof(yaffs_BlockInfo));
		blockInfo[i].blockState = YAFFS_BLOCK_STATE_EMPTY;
	}

	for (i = 0; i < objnum; i ++)
		blockInfo[(objInfo[i].startChunk - pageNum) / pageNum].hasShrinkHeader = 1;

	write(tmp_fd, blockInfo, part_blocks * sizeof(yaffs_BlockInfo));

	/* Write chunk bits */
	bitmap = (char *)blockInfo;
	memset(bitmap, 0xff, part_blocks * sizeof(yaffs_BlockInfo));
	pages = cp.nFreeChunks / chunkBitmapStride;
	page_sum = (part_blocks * pageNum) / chunkBitmapStride;
	memset(bitmap + (page_sum - pages), 0, pages);
	for (i = cp.nFreeChunks - pages * chunkBitmapStride; i > 0; i --)
		bitmap[page_sum - pages - 1] &= ~(1 << (chunkBitmapStride - i));
	write(tmp_fd, bitmap, part_blocks * sizeof(yaffs_BlockInfo));
	free(bitmap);

	return ret;
}

static void write_cp_object(int tmp_fd, yaffs_CheckpointObject *cp)
{
	int i = 0;
	unsigned int chunkoffset, tmp;
	__u16 chunk;

	chunkoffset = 0;
	write(tmp_fd, cp, sizeof(yaffs_CheckpointObject));
	if (cp->nDataChunks <= 0)
		return;

	chunk = 0;
	do {
		if (((chunkoffset >> YAFFS_TNODES_LEVEL0_BITS)
				<< YAFFS_TNODES_LEVEL0_BITS) == chunkoffset) {

			write(tmp_fd, &chunkoffset, sizeof(chunkoffset));
		}
		write(tmp_fd, &chunk, sizeof(chunk));
		if (!chunk)
			chunk = cp->hdrChunk;
		chunkoffset ++;
		chunk ++;
	} while (chunkoffset <= cp->nDataChunks);

	tmp = (chunkoffset >> YAFFS_TNODES_LEVEL0_BITS) << YAFFS_TNODES_LEVEL0_BITS;
	if (tmp != chunkoffset) {
		i = (1 << YAFFS_TNODES_LEVEL0_BITS) - chunkoffset + tmp;
		chunk = 0;
		for (; i > 0; i --)
			write(tmp_fd, &chunk, sizeof(chunk));
	}

	chunkoffset = ~0;
	write(tmp_fd, &chunkoffset, sizeof(chunkoffset));
}

static void yaffs_WriteCheckpointObjects(int tmp_fd, int objnum, struct objInfo *objInfo)
{
	yaffs_CheckpointObject cp;
	int buckets, i, j, start;
	unsigned objId;

	buckets = (objnum + BASE_ID) / YAFFS_NOBJECT_BUCKETS;
	for (j = 0, start = 2; j < YAFFS_NOBJECT_BUCKETS; j ++) {
		for (i = start; i <= buckets; i ++){
			objId = i * YAFFS_NOBJECT_BUCKETS + j;
			if (objId >= (objnum + BASE_ID))
				break;
			memset(&cp, 0, sizeof(cp));
			cp.structType = sizeof(cp);
			cp.objectId = objId;
			cp.parentId = objInfo[objId - BASE_ID].parentId;
			cp.hdrChunk = objInfo[objId - BASE_ID].startChunk;
			cp.variantType = objInfo[objId - BASE_ID].variantType;
			cp.renameAllowed = 1;
			cp.unlinkAllowed = 1;
			cp.nDataChunks = objInfo[objId - BASE_ID].chunks;
			if (cp.nDataChunks > 0)
				cp.fileSizeOrEquivalentObjectId = objInfo[objId - BASE_ID].filesize;
			else
				cp.fileSizeOrEquivalentObjectId = 0;
			write_cp_object(tmp_fd, &cp);
		}
		if (start > 1)
			start --;
		if (j <= YAFFS_OBJECTID_DELETED && j >= YAFFS_OBJECTID_ROOT) {
			objId = j;
			memset(&cp, 0, sizeof(cp));
			cp.structType = sizeof(cp);
			cp.objectId = objId;
			cp.variantType = YAFFS_OBJECT_TYPE_DIRECTORY;
			cp.fake = 1;
			cp.fileSizeOrEquivalentObjectId = 0;
			switch (j) {
			case YAFFS_OBJECTID_ROOT:
				cp.hdrChunk = pageNum;
				break;
			case YAFFS_OBJECTID_UNLINKED:
			case YAFFS_OBJECTID_DELETED:
				cp.parentId = 0;
				break;

			case YAFFS_OBJECTID_LOSTNFOUND:
				cp.parentId = YAFFS_OBJECTID_ROOT;
				break;
			}
			write_cp_object(tmp_fd, &cp);
		}
	}

	/* Dump end of list */
	memset(&cp, 0xFF, sizeof(yaffs_CheckpointObject));
	cp.structType = sizeof(cp);
	write_cp_object(tmp_fd, &cp);
}

static void usage(void)
{
	printf("usage: addcheckpoint image_file page_size page_num_per_block partition_size\n");
	exit(1);
}

static void read_header(int fd, yaffs_ObjectHeader *yoheader)
{
	int ret;
	lseek(fd, -0x808, SEEK_CUR);
	ret = read(fd, yoheader, sizeof(yaffs_ObjectHeader));
	if (ret < 0) {
		printf("read header fail\n");
		close(fd);
		exit(1);
	}
	lseek(fd, 0x808 - sizeof(yaffs_ObjectHeader), SEEK_CUR);
}

static struct objInfo *analyze_datachunks(int fd, int *objnum, unsigned int filesize)
{
	unsigned int objid, preid = 0, chunks;
	int id_remain = 500, match = 0, ret;
	__u16 indicator[4];
	yaffs_ObjectHeader yoheader;
	struct objInfo *objInfo, *tmp;

	objInfo = malloc(sizeof(struct objInfo) * id_remain);
	if (!objInfo) {
		printf("alloc mem fail, analyze exit\n");
		close(fd);
		exit(1);
	}
	/* we omit the first chunk for its special id as root */
	*objnum = 0;
	chunks = 1;
	lseek(fd, 0x1048, SEEK_SET);
	do {
		lseek(fd, -0x8, SEEK_CUR);
		ret = read(fd, indicator, sizeof(indicator));
		if (ret < 0) {
			printf("read indicator fail!!\n");
			close(fd);
			exit(1);
		}
		if (indicator[0] == YAFFS_LOWEST_SEQUENCE_NUMBER && indicator[1] == 0) {
			match = 1;
			objid = *((unsigned int *)&indicator[2]);
			if (objid < BASE_ID) {
				printf("It seems impossible, should check the image\n");
				close(fd);
				exit(1);
			}
			if (preid != objid) {
				read_header(fd, &yoheader);
				objInfo[objid - BASE_ID].chunks = 0;
				objInfo[objid - BASE_ID].parentId = yoheader.parentObjectId;
				objInfo[objid - BASE_ID].variantType = yoheader.type;
				objInfo[objid - BASE_ID].startChunk = chunks + pageNum;
				objInfo[objid - BASE_ID].filesize = yoheader.fileSize;

				id_remain --;
				*objnum  = *objnum + 1;
				preid = objid;
			}
			else
				objInfo[objid - BASE_ID].chunks ++;
			ret = lseek(fd, 0x840, SEEK_CUR);
			if (ret > filesize)
				match = 0;
			chunks ++;
			/* seems we should alloc more mem */
			if (id_remain == 0) {
				id_remain = 500;
				tmp = realloc(objInfo, (id_remain + *objnum) * sizeof(struct objInfo));
				if (!tmp) {
					printf("claim for more mem fail!!\n");
					close(fd);
					exit(1);
				}
				objInfo = tmp;
			}
		}
		else
			match = 0;
	} while(match);

	return objInfo;
}

int main(int argc, char *argv[])
{
	int fd, tmp_fd, objnum, ret;
	struct stat stats;
	char *endptr = NULL;
	unsigned char *buf, tmp;
	unsigned int val, count, Sum, Xor, file_blocks, cp_size, cp_start, cp_sum;
	struct objInfo *objInfo;

	printf("addcheckpoint: convert the ordinary yaffs image to the one"
	       " append with checkpoint\nCheckpoint version %d\nbuilt date:"
	       ""__DATE__"\n", YAFFS_CHECKPOINT_VERSION);
	if(argc != 5)
		usage();

	unlink(TMP_FILE);
	if(stat(argv[1],&stats) < 0)
	{
		printf("Could not stat %s\n",argv[1]);
		exit(1);
	}

	chunkSize = strtoul(argv[2], &endptr, 0);
	if (!endptr || *endptr != '\0') {
		printf("invalid page size 0x%x\n", chunkSize);
		usage();
	}

	pageNum = strtoul(argv[3], &endptr, 0);
	if (!endptr || *endptr != '\0') {
		printf("invalid page num per block 0x%x\n", pageNum);
		usage();
	}

	blocksize_shift = ffs(chunkSize * pageNum) - 1;
	spareSize = chunkSize / 32;
	val = strtoul(argv[4], &endptr, 0);
	if (!endptr || *endptr != '\0') {
		printf("invalid partition size 0x%x\n", val);
		usage();
	}

	if (val < stats.st_size) {
		printf("The image size exceed the partition size!\n");
		exit(1);
	}
	if (((val >> blocksize_shift) << blocksize_shift) != val) {
		printf("The partition size should be block size aligned!!\n");
		exit(1);
	}
	fd = open(argv[1], O_RDONLY);
	if(fd < 0) {
		printf("Could not open file %s\n",argv[1]);
		exit(1);
	}

	objInfo = analyze_datachunks(fd, &objnum, stats.st_size);
	close(fd);

	tmp_fd = open(TMP_FILE, O_RDWR | O_CREAT, 0777);
	if(tmp_fd < 0) {
		printf("Could not open tmp file %s\n", TMP_FILE);
		exit(1);
	}

	yaffs_WriteCheckpointValidityMarker(tmp_fd, 1);
	ret = yaffs_WriteCheckpointDevice(tmp_fd, val, stats.st_size, objnum, objInfo);
	if (ret < 0) {
		printf("write checkpoint device fail\n");
		close(tmp_fd);
		close(fd);
		exit(1);
	}
	cp_start =  ret * sizeof(yaffs_BlockInfo) + sizeof(yaffs_CheckpointValidity) + sizeof(yaffs_CheckpointDevice);
	yaffs_WriteCheckpointObjects(tmp_fd, objnum, objInfo);
	yaffs_WriteCheckpointValidityMarker(tmp_fd, 0);
	cp_size = lseek(fd, 0, SEEK_CUR);
	count = (cp_size + ((1 << blocksize_shift) - 1)) >> blocksize_shift;
	lseek(fd, cp_start, SEEK_SET);
	for (;count > 0; count --) {
		yaffs_BlockInfo blockInfo;
		memset(&blockInfo, 0, sizeof(yaffs_BlockInfo));
		blockInfo.blockState = YAFFS_BLOCK_STATE_CHECKPOINT;
		write(fd, &blockInfo, sizeof(yaffs_BlockInfo));
	}
	close(tmp_fd);

	buf = malloc(chunkSize);
	if (!buf) {
		printf("malloc tmp buf fail\n");
		exit(1);
	}
	tmp_fd = open(TMP_FILE, O_RDONLY);
	if(tmp_fd < 0) {
		printf("Could not open tmp file %s\n", TMP_FILE);
		exit(1);
	}
	fd = open(argv[1], O_APPEND | O_RDWR);
	if(fd < 0) {
		close(tmp_fd);
		printf("Could not open file %s\n",argv[1]);
		exit(1);
	}

	/* append 0xff to align with block size */
	count = ((1 << blocksize_shift) / chunkSize) * spareSize;
	count += (1 << blocksize_shift);
	val = (stats.st_size - 1 + count) / count;
	val *= count;
	if (val != stats.st_size) {
		val = val - stats.st_size;
		tmp = 0xff;
		while (val > 0) {
			ret = write(fd, &tmp, 1);
			val --;
		}
	}
	val = count = Sum = Xor = 0;
	file_blocks = stats.st_size / ((chunkSize + spareSize) * pageNum);
	if (((file_blocks >> blocksize_shift) << blocksize_shift) != file_blocks)
		file_blocks ++;

	memset(buf, 0, chunkSize);
	do {
		ret = read(tmp_fd, &buf[count - val], 1);
		if (ret <= 0)
			break;
		Sum += buf[count - val];
		Xor ^= buf[count - val];
		count ++;

		if (count >= (chunkSize + val)) {
			ret = write_chunk(fd, buf, file_blocks + 2, count / chunkSize, chunkSize);
			memset(buf, 0, chunkSize);
			val += chunkSize;
			if (((val >> blocksize_shift) << blocksize_shift) == val)
				file_blocks ++;
		}
	} while (ret > 0);

	cp_sum = (Sum << 8) | (Xor & 0xFF);
	*(unsigned int *)(&buf[count - val]) = cp_sum;
	count += 4;
	while(count > val) {
		write_chunk(fd, buf, file_blocks + 2, (count + chunkSize - 1) / chunkSize, chunkSize);
		val += chunkSize;
	}

	close(fd);
	close(tmp_fd);
	free(buf);
	exit(0);
}
