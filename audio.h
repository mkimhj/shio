// audio.h

void audioInit(void);
void audioStart(void);
void audioDeInit(void);
int16_t* audioGetMicData(void);
uint32_t audioGetBufferReleasedTime(void);