#include <cstdint>
#include <cmath>
#include <array>
#include <alsa/asoundlib.h>
int main() {
  snd_pcm_t *pcm = nullptr;
  if(snd_pcm_open(&pcm,"default",SND_PCM_STREAM_PLAYBACK,0)<0)return -1;
  if(snd_pcm_set_params(pcm,SND_PCM_FORMAT_S16,
    SND_PCM_ACCESS_RW_INTERLEAVED,1,44100,1,1)<0)return -1;
  std::array< int16_t, 1024 > buf;
  int pos = 0, ret;
  while(1) {
    for(int16_t &elem:buf)elem=sin(double(pos++)*440.*M_PI/44100.)*32767;
    if((ret=snd_pcm_writei(pcm,(const void*)buf.data(),buf.size()))<0)
      if(snd_pcm_recover(pcm,ret,0)<0)return -1;
  }
}
