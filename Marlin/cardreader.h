#ifndef CARDREADER_H
#define CARDREADER_H

#include "Marlin.h"

#ifdef SDSUPPORT

#include "SdFat.h"

#define SD_PATH_LENGTH 64  // longest file name or path that can be remembered

class CardReader
{
public:
  CardReader();

  void initsd();
  void release();
  void ls();
  void openFile(char* name, bool read, bool replace_current=true);
  void openLogFile(char* name);
  void removeFile(char* name);
  void closefile();
  void startFileprint();
  void pauseSDPrint();
  void getStatus();
  void printingHasFinished();
  void write_command(char *buf);
  // files auto[0-9].g on the sd card are performed in a row; the check is delayed
  // some seconds after power up so the host can connect quickly
  void checkautostart(bool force);

  FORCE_INLINE bool eof() { return sdpos >= filesize; }
  FORCE_INLINE int16_t get() { sdpos = file.curPosition(); return file.read(); }
  FORCE_INLINE void setIndex(long index) { sdpos = index; file.seekSet(index); }

  bool saving;
  bool logging;
  bool sdprinting;
  bool cardOK;

private:
  SdFile root, file;
  Sd2Card card;
  SdVolume volume;

  uint32_t filesize;
  uint32_t sdpos;

  char openName[SD_PATH_LENGTH];     // name the current file was opened with
  char parentName[SD_PATH_LENGTH];   // file that called the current one as a procedure
  uint32_t parentPos;                // position in the parent file
  bool inProcedure;

  unsigned long autostart_atmillis;
  bool autostart_stilltocheck;
  int8_t autostart_nr;               // number of the next autoN.g file, -1 when done

  bool openParentDir(char* name, SdFile& dir, char*& fname);
  void lsDive(const char *prepend, SdFile& dir);
};
extern CardReader card;

#define IS_SD_PRINTING (card.sdprinting)

#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT
#endif
