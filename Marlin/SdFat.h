/**
 * SdFat - minimal FAT16/FAT32 file system for the Delta firmware.
 *
 * Based on Arduino SdFat Library, Copyright (C) 2009 by William Greiman
 * (GPL v3). Reduced to what the SD card reader needs: open / read / write /
 * remove files with 8.3 names and directory listing.
 */
#ifndef SDFAT_H
#define SDFAT_H

#include "Sd2Card.h"

#ifdef SDSUPPORT

#define PACKED __attribute__((__packed__))

//------------------------------------------------------------------------------
// On disk structures

struct partitionTable {
  uint8_t  boot;
  uint8_t  beginHead;
  unsigned beginSector : 6;
  unsigned beginCylinderHigh : 2;
  uint8_t  beginCylinderLow;
  uint8_t  type;
  uint8_t  endHead;
  unsigned endSector : 6;
  unsigned endCylinderHigh : 2;
  uint8_t  endCylinderLow;
  uint32_t firstSector;
  uint32_t totalSectors;
} PACKED;
typedef struct partitionTable part_t;

struct masterBootRecord {
  uint8_t  codeArea[440];
  uint32_t diskSignature;
  uint16_t usuallyZero;
  part_t   part[4];
  uint8_t  mbrSig0;
  uint8_t  mbrSig1;
} PACKED;
typedef struct masterBootRecord mbr_t;

/** Boot sector, FAT32 layout. The leading fields are shared with FAT16. */
struct fat32_boot {
  uint8_t  jump[3];
  char     oemId[8];
  uint16_t bytesPerSector;
  uint8_t  sectorsPerCluster;
  uint16_t reservedSectorCount;
  uint8_t  fatCount;
  uint16_t rootDirEntryCount;
  uint16_t totalSectors16;
  uint8_t  mediaType;
  uint16_t sectorsPerFat16;
  uint16_t sectorsPerTrack;
  uint16_t headCount;
  uint32_t hidddenSectors;
  uint32_t totalSectors32;
  uint32_t sectorsPerFat32;
  uint16_t fat32Flags;
  uint16_t fat32Version;
  uint32_t fat32RootCluster;
  uint8_t  rest[464];
} PACKED;
typedef struct fat32_boot fat32_boot_t;

/** 32 byte directory entry */
struct directoryEntry {
  uint8_t  name[11];
  uint8_t  attributes;
  uint8_t  reservedNT;
  uint8_t  creationTimeTenths;
  uint16_t creationTime;
  uint16_t creationDate;
  uint16_t lastAccessDate;
  uint16_t firstClusterHigh;
  uint16_t lastWriteTime;
  uint16_t lastWriteDate;
  uint16_t firstClusterLow;
  uint32_t fileSize;
} PACKED;
typedef struct directoryEntry dir_t;

uint8_t const DIR_NAME_DELETED = 0XE5;
uint8_t const DIR_NAME_FREE = 0X00;
uint8_t const DIR_ATT_READ_ONLY = 0X01;
uint8_t const DIR_ATT_VOLUME_ID = 0X08;
uint8_t const DIR_ATT_DIRECTORY = 0X10;

static inline uint8_t DIR_IS_SUBDIR(const dir_t* dir) {
  return (dir->attributes & (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY)) == DIR_ATT_DIRECTORY;
}
static inline uint8_t DIR_IS_FILE_OR_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_VOLUME_ID) == 0;
}

// FAT end-of-chain marks
uint16_t const FAT16EOC_MIN = 0XFFF8;
uint32_t const FAT32EOC = 0X0FFFFFFF;
uint32_t const FAT32EOC_MIN = 0X0FFFFFF8;
uint32_t const FAT32MASK = 0X0FFFFFFF;

/** Default date (1 Jan 2000) and time (1 am) of created files */
uint16_t const FAT_DEFAULT_DATE = ((2000 - 1980) << 9) | (1 << 5) | 1;
uint16_t const FAT_DEFAULT_TIME = (1 << 11);

//------------------------------------------------------------------------------
// open() flags
uint8_t const O_READ   = 0X01;
uint8_t const O_WRITE  = 0X02;
uint8_t const O_APPEND = 0X04;  // every write goes to the end of the file
uint8_t const O_TRUNC  = 0X10;  // truncate to zero length on open
uint8_t const O_CREAT  = 0X40;  // create the file if it does not exist

//------------------------------------------------------------------------------
/** One 512 byte block cache shared by all files, viewed in different ways. */
union cache_t {
  uint8_t      data[512];
  uint16_t     fat16[256];
  uint32_t     fat32[128];
  dir_t        dir[16];
  mbr_t        mbr;
  fat32_boot_t fbs32;
};

//------------------------------------------------------------------------------
/** A FAT16 or FAT32 volume on an SD card. */
class SdVolume {
 public:
  SdVolume() : fatType_(0) {}
  /** Initialize the first valid partition, or an unpartitioned card. */
  bool init(Sd2Card* dev) { return init(dev, 1) || init(dev, 0); }

 private:
  friend class SdFile;

  static cache_t  cacheBuffer_;       // 512 byte cache for FAT or directory
  static uint32_t cacheBlockNumber_;  // block number of the cached block
  static Sd2Card* sdCard_;            // the card
  static bool     cacheDirty_;        // cache has to be written back
  static uint32_t cacheMirrorBlock_;  // block of the mirror FAT to write too

  uint32_t allocSearchStart_;   // start of the free cluster search
  uint8_t  blocksPerCluster_;
  uint32_t blocksPerFat_;
  uint32_t clusterCount_;
  uint8_t  clusterSizeShift_;
  uint32_t dataStartBlock_;
  uint8_t  fatCount_;
  uint32_t fatStartBlock_;
  uint8_t  fatType_;            // 16 or 32, zero if not initialized
  uint16_t rootDirEntryCount_;
  uint32_t rootDirStart_;       // block of the FAT16 root, cluster of the FAT32 root

  bool init(Sd2Card* dev, uint8_t part);

  uint32_t clusterStartBlock(uint32_t cluster) const {
    return dataStartBlock_ + ((cluster - 2) << clusterSizeShift_);
  }
  bool isEOC(uint32_t cluster) const {
    return cluster >= (fatType_ == 16 ? FAT16EOC_MIN : FAT32EOC_MIN);
  }

  static bool cacheFlush();
  static bool cacheRawBlock(uint32_t blockNumber, bool dirty);
  bool allocCluster(uint32_t* curCluster);
  bool chainSize(uint32_t beginCluster, uint32_t* size);
  bool fatGet(uint32_t cluster, uint32_t* value);
  bool fatPut(uint32_t cluster, uint32_t value);
  bool freeChain(uint32_t cluster);
};

//------------------------------------------------------------------------------
/** A file or directory (including the root directory) on a volume. */
class SdFile {
 public:
  SdFile() : writeError(false), type_(TYPE_CLOSED) {}
  ~SdFile() {if (isOpen()) close();}

  /** Set to true if a write() fails */
  bool writeError;

  bool close();
  /** Convert a directory entry name to "NAME.EXT" (at least 13 bytes needed). */
  static void dirName(const dir_t& dir, char* name);
  uint32_t curPosition() const {return curPosition_;}
  uint32_t fileSize() const {return fileSize_;}
  bool isOpen() const {return type_ != TYPE_CLOSED;}
  bool open(SdFile* dirFile, const char* path, uint8_t oflag);
  bool openRoot(SdVolume* vol);
  int16_t read();
  int16_t read(void* buf, uint16_t nbyte);
  int8_t readDir(dir_t* dir);
  bool remove();
  static bool remove(SdFile* dirFile, const char* path);
  void rewind() {seekSet(0);}
  bool seekSet(uint32_t pos);
  bool sync();
  int16_t write(const void* buf, uint16_t nbyte);
  int16_t write(const char* str) { return write(str, strlen(str)); }

 private:
  // values for type_
  static uint8_t const TYPE_CLOSED     = 0;
  static uint8_t const TYPE_NORMAL     = 1;
  static uint8_t const TYPE_ROOT_FIXED = 2;  // FAT16 root directory
  static uint8_t const TYPE_ROOT32     = 3;  // FAT32 root directory
  static uint8_t const TYPE_SUBDIR     = 4;
  static uint8_t const TYPE_MIN_DIR    = TYPE_ROOT_FIXED;

  // flags_ bits: access mode and append are kept from the open() flags
  static uint8_t const F_OFLAG = (O_READ | O_WRITE | O_APPEND);
  static uint8_t const F_FILE_DIR_DIRTY = 0X80;  // directory entry needs sync

  uint8_t   flags_;
  uint8_t   type_;
  uint32_t  curCluster_;    // cluster of the current position
  uint32_t  curPosition_;   // current position in bytes from the beginning
  uint32_t  dirBlock_;      // block of this file's directory entry
  uint8_t   dirIndex_;      // index of the directory entry in dirBlock_
  uint32_t  fileSize_;
  uint32_t  firstCluster_;
  SdVolume* vol_;

  bool isDir() const {return type_ >= TYPE_MIN_DIR;}
  bool isFile() const {return type_ == TYPE_NORMAL;}
  bool isRoot() const {return type_ == TYPE_ROOT_FIXED || type_ == TYPE_ROOT32;}

  bool addCluster();
  bool addDirCluster();
  dir_t* cacheDirEntry(bool dirty);
  static bool make83Name(const char* str, uint8_t* name, const char** ptr);
  bool open(SdFile* dirFile, const uint8_t dname[11], uint8_t oflag);
  bool openCachedEntry(uint8_t cacheIndex, uint8_t oflag);
  dir_t* readDirCache();
  bool truncate();
};

#endif  // SDSUPPORT
#endif  // SDFAT_H
