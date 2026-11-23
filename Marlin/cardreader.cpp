#include "cardreader.h"
#include "fan.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"


CardReader::CardReader()
{
  filesize = 0;
  sdpos = 0;
  sdprinting = false;
  cardOK = false;
  saving = false;
  logging = false;
  inProcedure = false;
  parentPos = 0;
  openName[0] = parentName[0] = 0;
  autostart_stilltocheck = true;
  autostart_nr = 0;
  autostart_atmillis = millis() + 5000;

  // power to SD reader
}

void CardReader::initsd()
{
  // M21 while a file is open: close it, the old cluster chain does not survive a volume re-init
  if (file.isOpen()) file.close();
  saving = logging = sdprinting = false;
  cardOK = false;
  if (root.isOpen()) root.close();

  #ifdef SDSLOW   // Configuration.h: slower SPI for a card that fails to init at full speed
    if (!card.init(SPI_HALF_SPEED, SDSS))
  #else
    if (!card.init(SPI_FULL_SPEED, SDSS))
  #endif
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
  }
  else if (!volume.init(&card))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
  }
  else if (!root.openRoot(&volume))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
  }
  else
  {
    cardOK = true;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
  }
}

void CardReader::release()
{
  sdprinting = false;
  cardOK = false;
}

void CardReader::startFileprint()
{
  if (cardOK) sdprinting = true;
}

void CardReader::pauseSDPrint()
{
  sdprinting = false;
}

//------------------------------------------------------------------------------
// Listing: prints all G-code files (*.G*) and, recursively, those in subdirectories.

void CardReader::lsDive(const char *prepend, SdFile& dir)
{
  dir_t p;
  char name[13];

  while (dir.readDir(&p) > 0)
  {
    if (p.name[0] == '_') continue;  // hidden files of other operating systems
    SdFile::dirName(p, name);

    if (DIR_IS_SUBDIR(&p))
    {
      char path[SD_PATH_LENGTH];
      if (strlen(prepend) + strlen(name) + 3 > sizeof(path)) continue;  // path too long
      // paths start with '/', a leading one is only added at the top level
      strcpy(path, *prepend ? prepend : "/");
      strcat(path, name);
      strcat(path, "/");

      SdFile sub;
      if (sub.open(&dir, name, O_READ))
      {
        lsDive(path, sub);
      }
      else
      {
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
        SERIAL_ECHOLN(name);
      }
    }
    else if (p.name[8] == 'G' && p.name[9] != '~')  // only .G* files, no safety copies
    {
      SERIAL_PROTOCOL(prepend);
      SERIAL_PROTOCOLLN(name);
    }
  }
}

void CardReader::ls()
{
  root.rewind();
  lsDive("", root);
}

//------------------------------------------------------------------------------
// Files

void CardReader::openFile(char* name, bool read, bool replace_current/*=true*/)
{
  if (!cardOK) return;

  if (file.isOpen())  // replacing the current file by a new file, or a procedure call
  {
    if (!replace_current)
    {
      if (inProcedure)
      {
        SERIAL_ERROR_START;
        SERIAL_ERRORPGM("trying to call sub-gcode files with too many levels. MAX level is:");
        SERIAL_ERRORLN(1);
        kill();
        return;
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("SUBROUTINE CALL target:\"");
      SERIAL_ECHO(name);
      SERIAL_ECHOPGM("\" parent:\"");
      SERIAL_ECHO(openName);
      SERIAL_ECHOPGM("\" pos");
      SERIAL_ECHOLN(sdpos);
      // remember the calling file and where to continue
      strcpy(parentName, openName);
      parentPos = sdpos;
      inProcedure = true;
    }
    else
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("Now doing file: ");
      SERIAL_ECHOLN(name);
    }
    file.close();
  }
  else  // opening a fresh file
  {
    inProcedure = false;  // in case the user cancelled a print while in a procedure
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Now fresh file: ");
    SERIAL_ECHOLN(name);
  }
  sdprinting = false;

  // the path is relative to the root directory; subdirectories are separated by '/'
  if (read)
  {
    if (file.open(&root, name, O_READ))
    {
      filesize = file.fileSize();
      sdpos = 0;
      strncpy(openName, name, sizeof(openName) - 1);
      openName[sizeof(openName) - 1] = 0;
      SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
      SERIAL_PROTOCOL(name);
      SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
      SERIAL_PROTOCOLLN(filesize);
      SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
      return;
    }
  }
  else if (file.open(&root, name, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
  {
    saving = true;
    SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
    SERIAL_PROTOCOLLN(name);
    return;
  }
  SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
  SERIAL_PROTOCOL(name);
  SERIAL_PROTOCOLLNPGM(".");
}

void CardReader::openLogFile(char* name)
{
  logging = true;
  openFile(name, false);
}

void CardReader::removeFile(char* name)
{
  if (!cardOK) return;
  file.close();
  sdprinting = false;

  if (SdFile::remove(&root, name))
  {
    SERIAL_PROTOCOLPGM("File deleted:");
    SERIAL_PROTOCOLLN(name);
    sdpos = 0;
  }
  else
  {
    SERIAL_PROTOCOLPGM("Deletion failed, File: ");
    SERIAL_PROTOCOL(name);
    SERIAL_PROTOCOLLNPGM(".");
  }
}

void CardReader::closefile()
{
  file.close();  // also writes the directory entry and the cache
  saving = false;
  logging = false;
}

void CardReader::getStatus()
{
  if (cardOK)
  {
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(sdpos);
    SERIAL_PROTOCOLPGM("/");
    SERIAL_PROTOCOLLN(filesize);
  }
  else
  {
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
  }
}

// Write a command received while saving to the file. Line number and checksum
// ("N123 command *45") are stripped.
void CardReader::write_command(char *buf)
{
  char* begin = buf;
  char* end = buf + strlen(buf) - 1;

  char* npos = strchr(buf, 'N');
  if (npos)
  {
    char* space = strchr(npos, ' ');
    char* star = strchr(npos, '*');
    if (space && star)
    {
      begin = space + 1;
      end = star - 1;
    }
  }

  file.writeError = false;
  if (end >= begin) file.write(begin, end - begin + 1);
  file.write("\r\n", 2);
  if (file.writeError)
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
  }
}

//------------------------------------------------------------------------------
// Autostart: auto0.g, auto1.g, ... in the root directory are printed in a row.

void CardReader::checkautostart(bool force)
{
  if (!force)
  {
    if (!autostart_stilltocheck) return;
    if (autostart_atmillis < millis()) return;
  }
  autostart_stilltocheck = false;
  if (autostart_nr < 0) return;

  if (!cardOK)
  {
    initsd();
    if (!cardOK) return;
  }

  dir_t p;
  root.rewind();
  while (root.readDir(&p) > 0)
  {
    // "AUTOn   G  " but not safety copies like "AUTOn   G~ "
    if (memcmp(p.name, "AUTO", 4) == 0 && p.name[4] == '0' + autostart_nr &&
        p.name[5] == ' ' && p.name[8] == 'G' && p.name[9] == ' ')
    {
      char cmd[12];
      sprintf_P(cmd, PSTR("M23 auto%d.g"), autostart_nr);
      enquecommand(cmd);
      enquecommand_P(PSTR("M24"));
      autostart_nr = autostart_nr < 9 ? autostart_nr + 1 : -1;
      return;
    }
  }
  autostart_nr = -1;  // no more auto files
}

bool CardReader::startNewest()
{
  if (!cardOK)
  {
    initsd();
    if (!cardOK) return false;
  }
  dir_t p;
  char best[13];
  uint32_t best_stamp = 0;
  bool found = false;
  root.rewind();
  while (root.readDir(&p) > 0)
  {
    if (p.name[0] == '_' || DIR_IS_SUBDIR(&p) || p.name[8] != 'G' || p.name[9] == '~') continue;
    uint32_t stamp = ((uint32_t)p.lastWriteDate << 16) | p.lastWriteTime;
    if (!found || stamp >= best_stamp)
    {
      found = true;
      best_stamp = stamp;
      SdFile::dirName(p, best);
    }
  }
  if (!found) return false;
  char cmd[20];
  strcpy_P(cmd, PSTR("M23 "));
  strcat(cmd, best);
  enquecommand(cmd);
  enquecommand_P(PSTR("M24"));
  return true;
}

//------------------------------------------------------------------------------
void CardReader::printingHasFinished()
{
  st_synchronize();
  file.close();
  if (inProcedure)  // continue the file that called the current one as a procedure
  {
    inProcedure = false;
    openFile(parentName, true, true);
    setIndex(parentPos);
    startFileprint();
  }
  else
  {
    quickStop();
    sdprinting = false;
    fan_start_cooldown();   // the print is over: cool the hotend at full fan, forget the M106 of the print
    if (SD_FINISHED_STEPPERRELEASE)
    {
      enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    }
  }
}

