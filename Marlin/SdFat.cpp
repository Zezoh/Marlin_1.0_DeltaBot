/**
 * SdFat - minimal FAT16/FAT32 file system for the Delta firmware.
 *
 * Based on Arduino SdFat Library, Copyright (C) 2009 by William Greiman
 * (GPL v3).
 */
#include "SdFat.h"


cache_t  SdVolume::cacheBuffer_;
uint32_t SdVolume::cacheBlockNumber_;
Sd2Card* SdVolume::sdCard_;
bool     SdVolume::cacheDirty_;
uint32_t SdVolume::cacheMirrorBlock_;

//==============================================================================
// SdVolume

/** Write the cached block (and its FAT mirror) back to the card if dirty. */
bool SdVolume::cacheFlush() {
  if (cacheDirty_) {
    if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data)) return false;
    if (cacheMirrorBlock_) {
      if (!sdCard_->writeBlock(cacheMirrorBlock_, cacheBuffer_.data)) return false;
      cacheMirrorBlock_ = 0;
    }
    cacheDirty_ = false;
  }
  return true;
}

/** Make a block the cached block. */
bool SdVolume::cacheRawBlock(uint32_t blockNumber, bool dirty) {
  if (cacheBlockNumber_ != blockNumber) {
    if (!cacheFlush()) return false;
    if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data)) return false;
    cacheBlockNumber_ = blockNumber;
  }
  if (dirty) cacheDirty_ = true;
  return true;
}

/** Get the FAT entry of a cluster. */
bool SdVolume::fatGet(uint32_t cluster, uint32_t* value) {
  if (cluster > (clusterCount_ + 1)) return false;
  uint32_t lba = fatStartBlock_ + (cluster >> (fatType_ == 16 ? 8 : 7));
  if (!cacheRawBlock(lba, false)) return false;
  *value = fatType_ == 16 ? cacheBuffer_.fat16[cluster & 0XFF]
                          : cacheBuffer_.fat32[cluster & 0X7F] & FAT32MASK;
  return true;
}

/** Set the FAT entry of a cluster (in all FAT copies). */
bool SdVolume::fatPut(uint32_t cluster, uint32_t value) {
  if (cluster < 2 || cluster > (clusterCount_ + 1)) return false;
  uint32_t lba = fatStartBlock_ + (cluster >> (fatType_ == 16 ? 8 : 7));
  if (!cacheRawBlock(lba, true)) return false;
  if (fatType_ == 16) {
    cacheBuffer_.fat16[cluster & 0XFF] = value;
  } else {
    cacheBuffer_.fat32[cluster & 0X7F] = value;
  }
  if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
  return true;
}

/** Allocate one free cluster and link it after curCluster (if not zero). */
bool SdVolume::allocCluster(uint32_t* curCluster) {
  uint32_t c = *curCluster ? *curCluster + 1 : allocSearchStart_;
  for (uint32_t n = 0; n < clusterCount_; n++, c++) {
    if (c > clusterCount_ + 1) c = 2;
    uint32_t f;
    if (!fatGet(c, &f)) return false;
    if (f == 0) {
      if (!fatPut(c, FAT32EOC)) return false;
      if (*curCluster) {
        if (!fatPut(*curCluster, c)) return false;
      } else {
        allocSearchStart_ = c + 1;
      }
      *curCluster = c;
      return true;
    }
  }
  return false;
}

/** Size in bytes of a cluster chain. */
bool SdVolume::chainSize(uint32_t cluster, uint32_t* size) {
  uint32_t s = 0;
  do {
    if (!fatGet(cluster, &cluster)) return false;
    s += 512UL << clusterSizeShift_;
  } while (!isEOC(cluster));
  *size = s;
  return true;
}

/** Free a cluster chain. */
bool SdVolume::freeChain(uint32_t cluster) {
  uint32_t next;
  allocSearchStart_ = 2;
  do {
    if (!fatGet(cluster, &next) || !fatPut(cluster, 0)) return false;
    cluster = next;
  } while (!isEOC(cluster));
  return true;
}

/** Initialize a volume. part is 1..4 for a partition or 0 for no partition table. */
bool SdVolume::init(Sd2Card* dev, uint8_t part) {
  uint32_t volumeStartBlock = 0;
  sdCard_ = dev;
  fatType_ = 0;
  allocSearchStart_ = 2;
  cacheDirty_ = false;
  cacheMirrorBlock_ = 0;
  cacheBlockNumber_ = 0XFFFFFFFF;

  if (part) {
    if (part > 4 || !cacheRawBlock(0, false)) return false;
    part_t* p = &cacheBuffer_.mbr.part[part - 1];
    if ((p->boot & 0X7F) != 0 || p->totalSectors < 100 || p->firstSector == 0) {
      return false;  // not a valid partition
    }
    volumeStartBlock = p->firstSector;
  }
  if (!cacheRawBlock(volumeStartBlock, false)) return false;
  fat32_boot_t* fbs = &cacheBuffer_.fbs32;
  if (fbs->bytesPerSector != 512 || fbs->fatCount == 0 ||
      fbs->reservedSectorCount == 0 || fbs->sectorsPerCluster == 0) {
    return false;  // not a FAT volume
  }
  fatCount_ = fbs->fatCount;
  blocksPerCluster_ = fbs->sectorsPerCluster;
  clusterSizeShift_ = 0;
  while (blocksPerCluster_ != (1 << clusterSizeShift_)) {
    if (clusterSizeShift_++ > 7) return false;  // not a power of two
  }
  blocksPerFat_ = fbs->sectorsPerFat16 ? fbs->sectorsPerFat16 : fbs->sectorsPerFat32;
  fatStartBlock_ = volumeStartBlock + fbs->reservedSectorCount;
  rootDirEntryCount_ = fbs->rootDirEntryCount;
  rootDirStart_ = fatStartBlock_ + fbs->fatCount * blocksPerFat_;
  dataStartBlock_ = rootDirStart_ + ((32 * fbs->rootDirEntryCount + 511) / 512);
  uint32_t totalBlocks = fbs->totalSectors16 ? fbs->totalSectors16 : fbs->totalSectors32;
  clusterCount_ = (totalBlocks - (dataStartBlock_ - volumeStartBlock)) >> clusterSizeShift_;

  if (clusterCount_ < 4085) {
    return false;  // FAT12 is not supported
  } else if (clusterCount_ < 65525) {
    fatType_ = 16;
  } else {
    rootDirStart_ = fbs->fat32RootCluster;
    fatType_ = 32;
  }
  return true;
}

//==============================================================================
// SdFile

/** Add a cluster to the end of the file. */
bool SdFile::addCluster() {
  if (!vol_->allocCluster(&curCluster_)) return false;
  if (firstCluster_ == 0) {
    firstCluster_ = curCluster_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  return true;
}

/** Add a cleared cluster to a directory. */
bool SdFile::addDirCluster() {
  if (fileSize_ / sizeof(dir_t) >= 0XFFFF) return false;  // directory is full
  if (!addCluster() || !SdVolume::cacheFlush()) return false;
  uint32_t block = vol_->clusterStartBlock(curCluster_);
  SdVolume::cacheBlockNumber_ = block;
  SdVolume::cacheDirty_ = true;
  memset(SdVolume::cacheBuffer_.data, 0, 512);
  for (uint8_t i = 1; i < vol_->blocksPerCluster_; i++) {
    if (!SdVolume::sdCard_->writeBlock(block + i, SdVolume::cacheBuffer_.data)) return false;
  }
  fileSize_ += 512UL << vol_->clusterSizeShift_;
  return true;
}

/** Cache the block of this file's directory entry and return the entry. */
dir_t* SdFile::cacheDirEntry(bool dirty) {
  return SdVolume::cacheRawBlock(dirBlock_, dirty) ? SdVolume::cacheBuffer_.dir + dirIndex_ : 0;
}

bool SdFile::close() {
  bool rtn = sync();
  type_ = TYPE_CLOSED;
  return rtn;
}

/** Write the directory entry (if changed) and the cache to the card. */
bool SdFile::sync() {
  if (!isOpen()) {
    writeError = true;
    return false;
  }
  if (flags_ & F_FILE_DIR_DIRTY) {
    dir_t* d = cacheDirEntry(true);
    if (!d || d->name[0] == DIR_NAME_DELETED) {
      writeError = true;
      return false;
    }
    if (!isDir()) d->fileSize = fileSize_;
    d->firstClusterLow = firstCluster_ & 0XFFFF;
    d->firstClusterHigh = firstCluster_ >> 16;
    flags_ &= ~F_FILE_DIR_DIRTY;
  }
  return SdVolume::cacheFlush();
}

/** Get the 8.3 name ("NAME.EXT") of the file, or "/" for the root. */
bool SdFile::getFilename(char* name) {
  if (!isOpen()) return false;
  if (isRoot()) {
    name[0] = '/';
    name[1] = '\0';
    return true;
  }
  dir_t* p = cacheDirEntry(false);
  if (!p) return false;
  dirName(*p, name);
  return true;
}

void SdFile::dirName(const dir_t& dir, char* name) {
  uint8_t j = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ') continue;
    if (i == 8) name[j++] = '.';
    name[j++] = dir.name[i];
  }
  name[j] = 0;
}

/** Convert a path element into an 8.3 name. Stops at '/' or end of string. */
bool SdFile::make83Name(const char* str, uint8_t* name, const char** ptr) {
  uint8_t n = 7;  // max index of the current part
  uint8_t i = 0;
  memset(name, ' ', 11);
  while (*str != '\0' && *str != '/') {
    uint8_t c = *str++;
    if (c == '.') {
      if (n == 10) return false;  // only one dot allowed
      n = 10;
      i = 8;
    } else {
      if (strchr_P(PSTR("|<>^+=?/[];,*\"\\"), c)) return false;
      if (i > n || c < 0X21 || c > 0X7E) return false;
      name[i++] = (c < 'a' || c > 'z') ? c : (c + ('A' - 'a'));
    }
  }
  *ptr = str;
  return name[0] != ' ';
}

/** Open a file or directory by path relative to dirFile (or to the root if it starts with '/'). */
bool SdFile::open(SdFile* dirFile, const char* path, uint8_t oflag) {
  uint8_t dname[11];
  SdFile dir1, dir2;
  SdFile* parent = dirFile;
  SdFile* sub = &dir1;

  if (!dirFile || isOpen()) return false;
  if (*path == '/') {
    while (*path == '/') path++;
    if (!dirFile->isRoot()) {
      if (!dir2.openRoot(dirFile->vol_)) return false;
      parent = &dir2;
    }
  }
  while (true) {
    if (!make83Name(path, dname, &path)) return false;
    while (*path == '/') path++;
    if (!*path) break;
    if (!sub->open(parent, dname, O_READ)) return false;
    if (parent != dirFile) parent->close();
    parent = sub;
    sub = parent != &dir1 ? &dir1 : &dir2;
  }
  return open(parent, dname, oflag);
}

/** Open (or create with O_CREAT) the entry with the 8.3 name dname in dirFile. */
bool SdFile::open(SdFile* dirFile, const uint8_t dname[11], uint8_t oflag) {
  bool emptyFound = false;
  bool fileFound = false;
  uint8_t index;
  dir_t* p;

  vol_ = dirFile->vol_;
  dirFile->rewind();
  while (dirFile->curPosition_ < dirFile->fileSize_) {
    index = 0XF & (dirFile->curPosition_ >> 5);
    p = dirFile->readDirCache();
    if (!p) return false;
    if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED) {
      if (!emptyFound) {
        dirBlock_ = SdVolume::cacheBlockNumber_;
        dirIndex_ = index;
        emptyFound = true;
      }
      if (p->name[0] == DIR_NAME_FREE) break;
    } else if (!memcmp(dname, p->name, 11)) {
      fileFound = true;
      break;
    }
  }

  if (!fileFound) {
    // create the file
    if (!(oflag & O_CREAT) || !(oflag & O_WRITE)) return false;
    if (emptyFound) {
      index = dirIndex_;
      p = cacheDirEntry(true);
      if (!p) return false;
    } else {
      // no free entry: extend the directory (not possible for a FAT16 root)
      if (dirFile->type_ == TYPE_ROOT_FIXED || !dirFile->addDirCluster()) return false;
      p = SdVolume::cacheBuffer_.dir;
      index = 0;
    }
    memset(p, 0, sizeof(dir_t));
    memcpy(p->name, dname, 11);
    p->creationDate = p->lastAccessDate = p->lastWriteDate = FAT_DEFAULT_DATE;
    p->creationTime = p->lastWriteTime = FAT_DEFAULT_TIME;
    if (!SdVolume::cacheFlush()) return false;
  }
  return openCachedEntry(index, oflag);
}

/** Open the entry dirIndex of the cached directory block. */
bool SdFile::openCachedEntry(uint8_t dirIndex, uint8_t oflag) {
  dir_t* p = &SdVolume::cacheBuffer_.dir[dirIndex];

  // read only files and directories can not be opened for write
  if ((p->attributes & (DIR_ATT_READ_ONLY | DIR_ATT_DIRECTORY)) && (oflag & (O_WRITE | O_TRUNC))) {
    return false;
  }
  dirBlock_ = SdVolume::cacheBlockNumber_;
  dirIndex_ = dirIndex;
  firstCluster_ = ((uint32_t)p->firstClusterHigh << 16) | p->firstClusterLow;

  if (DIR_IS_SUBDIR(p)) {
    if (!vol_->chainSize(firstCluster_, &fileSize_)) return false;
    type_ = TYPE_SUBDIR;
  } else if (!DIR_IS_FILE_OR_SUBDIR(p)) {
    return false;
  } else {
    fileSize_ = p->fileSize;
    type_ = TYPE_NORMAL;
  }
  flags_ = oflag & F_OFLAG;
  curCluster_ = 0;
  curPosition_ = 0;
  if ((oflag & O_TRUNC) && !truncate()) {
    type_ = TYPE_CLOSED;
    return false;
  }
  return true;
}

/** Open the root directory of a volume. */
bool SdFile::openRoot(SdVolume* vol) {
  if (isOpen()) return false;
  if (vol->fatType_ == 16) {
    type_ = TYPE_ROOT_FIXED;
    firstCluster_ = 0;
    fileSize_ = 32UL * vol->rootDirEntryCount_;
  } else if (vol->fatType_ == 32) {
    type_ = TYPE_ROOT32;
    firstCluster_ = vol->rootDirStart_;
    if (!vol->chainSize(firstCluster_, &fileSize_)) return false;
  } else {
    return false;
  }
  vol_ = vol;
  flags_ = O_READ;
  curCluster_ = 0;
  curPosition_ = 0;
  dirBlock_ = 0;
  dirIndex_ = 0;
  return true;
}

int16_t SdFile::read() {
  uint8_t b;
  return read(&b, 1) == 1 ? b : -1;
}

/** Read up to nbyte bytes. Returns the number of bytes read or -1 on error. */
int16_t SdFile::read(void* buf, uint16_t nbyte) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);

  if (!isOpen() || !(flags_ & O_READ)) return -1;
  if (nbyte > fileSize_ - curPosition_) nbyte = fileSize_ - curPosition_;

  uint16_t toRead = nbyte;
  while (toRead > 0) {
    uint16_t offset = curPosition_ & 0X1FF;  // offset in block
    uint32_t block;
    if (type_ == TYPE_ROOT_FIXED) {
      block = vol_->rootDirStart_ + (curPosition_ >> 9);
    } else {
      uint8_t blockOfCluster = (curPosition_ >> 9) & (vol_->blocksPerCluster_ - 1);
      if (offset == 0 && blockOfCluster == 0) {
        // start of a new cluster
        if (curPosition_ == 0) {
          curCluster_ = firstCluster_;
        } else if (!vol_->fatGet(curCluster_, &curCluster_)) {
          return -1;
        }
      }
      block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    }
    uint16_t n = 512 - offset;
    if (n > toRead) n = toRead;

    if (n == 512 && block != SdVolume::cacheBlockNumber_) {
      // whole block that is not cached: read directly to the caller's buffer
      if (!SdVolume::sdCard_->readBlock(block, dst)) return -1;
    } else {
      if (!SdVolume::cacheRawBlock(block, false)) return -1;
      memcpy(dst, SdVolume::cacheBuffer_.data + offset, n);
    }
    dst += n;
    curPosition_ += n;
    toRead -= n;
  }
  return nbyte;
}

/**
 * Read the next file or directory entry (long file name entries are skipped).
 * Returns the entry size (32), zero at the end of the directory, -1 on error.
 */
int8_t SdFile::readDir(dir_t* dir) {
  if (!isDir() || (0X1F & curPosition_)) return -1;
  while (true) {
    int16_t n = read(dir, sizeof(dir_t));
    if (n != sizeof(dir_t)) return n == 0 ? 0 : -1;
    if (dir->name[0] == DIR_NAME_FREE) return 0;
    if (dir->name[0] == DIR_NAME_DELETED || dir->name[0] == '.') continue;
    if (DIR_IS_FILE_OR_SUBDIR(dir)) return n;
  }
}

/** Return the directory entry at the current position and advance by one entry. */
dir_t* SdFile::readDirCache() {
  if (!isDir()) return 0;
  uint8_t i = (curPosition_ >> 5) & 0XF;
  if (read() < 0) return 0;  // caches the block
  curPosition_ += 31;
  return SdVolume::cacheBuffer_.dir + i;
}

/** Free all clusters of the file. */
bool SdFile::truncate() {
  if (!isFile() || !(flags_ & O_WRITE)) return false;
  if (fileSize_ == 0) return true;
  if (!vol_->freeChain(firstCluster_)) return false;
  firstCluster_ = 0;
  fileSize_ = 0;
  curCluster_ = 0;
  curPosition_ = 0;
  flags_ |= F_FILE_DIR_DIRTY;
  return sync();
}

/** Delete the open file. The file must be opened with O_WRITE. */
bool SdFile::remove() {
  if (!truncate()) return false;
  dir_t* d = cacheDirEntry(true);
  if (!d) return false;
  d->name[0] = DIR_NAME_DELETED;
  type_ = TYPE_CLOSED;
  return SdVolume::cacheFlush();
}

/** Delete the file path in dirFile. */
bool SdFile::remove(SdFile* dirFile, const char* path) {
  SdFile file;
  return file.open(dirFile, path, O_WRITE) && file.remove();
}

/** Set the file position. Returns false if pos is beyond the end of the file. */
bool SdFile::seekSet(uint32_t pos) {
  if (!isOpen() || pos > fileSize_) return false;
  if (type_ == TYPE_ROOT_FIXED) {
    curPosition_ = pos;
    return true;
  }
  if (pos == 0) {
    curCluster_ = 0;
    curPosition_ = 0;
    return true;
  }
  // number of clusters to follow from the current cluster
  uint8_t shift = vol_->clusterSizeShift_ + 9;
  uint32_t nCur = (curPosition_ - 1) >> shift;
  uint32_t nNew = (pos - 1) >> shift;
  if (nNew < nCur || curPosition_ == 0) {
    curCluster_ = firstCluster_;  // restart from the beginning of the chain
  } else {
    nNew -= nCur;
  }
  while (nNew--) {
    if (!vol_->fatGet(curCluster_, &curCluster_)) return false;
  }
  curPosition_ = pos;
  return true;
}

/** Write nbyte bytes at the current position (or at the end with O_APPEND). */
int16_t SdFile::write(const void* buf, uint16_t nbyte) {
  const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);
  uint16_t nToWrite = nbyte;

  if (!isFile() || !(flags_ & O_WRITE)) goto fail;
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_ && !seekSet(fileSize_)) goto fail;

  while (nToWrite > 0) {
    uint8_t blockOfCluster = (curPosition_ >> 9) & (vol_->blocksPerCluster_ - 1);
    uint16_t blockOffset = curPosition_ & 0X1FF;
    if (blockOfCluster == 0 && blockOffset == 0) {
      // start of a new cluster
      if (curCluster_ == 0) {
        if (firstCluster_ == 0) {
          if (!addCluster()) goto fail;
        } else {
          curCluster_ = firstCluster_;
        }
      } else {
        uint32_t next;
        if (!vol_->fatGet(curCluster_, &next)) goto fail;
        if (vol_->isEOC(next)) {
          if (!addCluster()) goto fail;
        } else {
          curCluster_ = next;
        }
      }
    }
    uint16_t n = 512 - blockOffset;
    if (n > nToWrite) n = nToWrite;
    uint32_t block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;

    if (n == 512) {
      // whole block: write directly, invalidating the cache if it holds this block
      if (SdVolume::cacheBlockNumber_ == block) SdVolume::cacheBlockNumber_ = 0XFFFFFFFF;
      if (!SdVolume::sdCard_->writeBlock(block, src)) goto fail;
    } else {
      if (blockOffset == 0 && curPosition_ >= fileSize_) {
        // new block past the end of the file: no need to read it
        if (!SdVolume::cacheFlush()) goto fail;
        SdVolume::cacheBlockNumber_ = block;
        SdVolume::cacheDirty_ = true;
      } else if (!SdVolume::cacheRawBlock(block, true)) {
        goto fail;
      }
      memcpy(SdVolume::cacheBuffer_.data + blockOffset, src, n);
    }
    curPosition_ += n;
    src += n;
    nToWrite -= n;
  }
  if (curPosition_ > fileSize_) {
    fileSize_ = curPosition_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  return nbyte;

 fail:
  writeError = true;
  return -1;
}

int16_t SdFile::write(const char* str) {
  return write(str, strlen(str));
}

