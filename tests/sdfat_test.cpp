#include "SdFat.h"
#include <assert.h>
#include <vector>
#include <string>
static Sd2Card card; static SdVolume vol; static SdFile root;
static std::vector<std::string> ls(SdFile& d){ std::vector<std::string> v; dir_t p; char n[13]; d.rewind();
  while(d.readDir(&p)>0){ SdFile::dirName(p,n); v.push_back(std::string(n)+(DIR_IS_SUBDIR(&p)?"/":"")); } return v; }
int main(int argc,char**argv){ // usage: sdfat_test <fat image> <reference file of big.g> <output image>
  FILE*f=fopen(argv[1],"rb"); fseek(f,0,SEEK_END); long sz=ftell(f); rewind(f);
  card.img=(uint8_t*)malloc(sz); card.blocks=sz/512; fread(card.img,1,sz,f); fclose(f);
  assert(vol.init((Sd2Card*)&card)); assert(root.openRoot(&vol));
  auto l=ls(root); for(auto&s:l) printf("root: %s\n",s.c_str());
  // read existing file, compare with content
  SdFile a; assert(a.open(&root,"/dir/sub/deep.g",O_READ)); printf("deep size %u\n",a.fileSize());
  SdFile b; assert(b.open(&root,"big.g",O_READ)); uint32_t n=b.fileSize(); printf("big size %u\n",n);
  std::vector<uint8_t> got(n); uint32_t pos=0; // byte-wise read like CardReader::get()
  int c; while((c=b.read())>=0) got[pos++]=c; assert(pos==n);
  FILE*r=fopen(argv[2],"rb"); std::vector<uint8_t> ref(n); fread(ref.data(),1,n,r); fclose(r); assert(got==ref); puts("big.g bytewise read OK");
  // seek + bulk read
  assert(b.seekSet(70000)); uint8_t buf[3000]; assert(b.read(buf,3000)==3000); assert(!memcmp(buf,&ref[70000],3000)); puts("seek/bulk OK");
  assert(!b.seekSet(n+1)); b.close();
  // create + write file in root, with append, multi-cluster
  SdFile w; assert(w.open(&root,"NEW.G",O_CREAT|O_APPEND|O_WRITE|O_TRUNC));
  std::string line; for(int i=0;i<20000;i++){ char t[40]; snprintf(t,40,"G1 X%d Y%d\r\n",i,i*2); line+=t; w.write(t); }
  assert(!w.writeError); assert(w.close());
  SdFile rd; assert(rd.open(&root,"new.g",O_READ)); assert(rd.fileSize()==line.size());
  std::vector<char> rb(line.size()); assert(rd.read(rb.data(),rb.size())==(int)rb.size()||line.size()>32767); 
  { rd.rewind(); size_t off=0; while(off<line.size()){ int k=rd.read(&rb[off],(line.size()-off>4096)?4096:line.size()-off); assert(k>0); off+=k;} }
  assert(std::string(rb.begin(),rb.end())==line); puts("write/read back OK");
  // overwrite with O_TRUNC then small write
  SdFile w2; assert(w2.open(&root,"NEW.G",O_CREAT|O_APPEND|O_WRITE|O_TRUNC)); w2.write("hello\r\n"); w2.close();
  SdFile rd2; assert(rd2.open(&root,"NEW.G",O_READ)); assert(rd2.fileSize()==7); rd2.close(); puts("trunc OK");
  // create in subdir
  SdFile w3; assert(w3.open(&root,"/dir/sub/made.g",O_CREAT|O_WRITE|O_APPEND)); w3.write("x\r\n"); w3.close();
  SdFile d1; assert(d1.open(&root,"dir",O_READ)); SdFile d2; assert(d2.open(&d1,"sub",O_READ)); l=ls(d2); for(auto&s:l) printf("sub: %s\n",s.c_str());
  // remove
  assert(SdFile::remove(&root,"NEW.G")); SdFile x; assert(!x.open(&root,"NEW.G",O_READ)); puts("remove OK");
  assert(!SdFile::remove(&root,"nonexistent.g"));
  // fill many files in root to force directory growth (FAT32) or fail gracefully (FAT16)
  int created=0; for(int i=0;i<600;i++){ char nm[16]; snprintf(nm,16,"F%d.G",i); SdFile q; if(!q.open(&root,nm,O_CREAT|O_WRITE)) break; q.write("a"); q.close(); created++; }
  printf("created %d small files\n",created);
  for(int i=0;i<created;i+=97){ char nm[16]; snprintf(nm,16,"f%d.g",i); SdFile q; assert(q.open(&root,nm,O_READ)); assert(q.fileSize()==1);} 
  FILE*o=fopen(argv[3],"wb"); fwrite(card.img,1,sz,o); fclose(o); printf("reads=%u writes=%u\n",card.reads,card.writes); puts("ALL OK"); }
