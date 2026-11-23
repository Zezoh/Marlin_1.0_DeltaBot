#pragma once
struct CardReader {
  bool sdprinting = false, paused = false;
  bool isPaused() { return paused; }
  void pauseSDPrint()
  {
    sdprinting = false;
    paused = true;
  }
  void startFileprint()
  {
    sdprinting = true;
    paused = false;
  }
};
extern CardReader card;
