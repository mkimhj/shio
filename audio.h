// audio.h

typedef enum {
  EVENT_COMP_NONE = 0,
  EVENT_COMP_OVER_SAMPLE,
  EVENT_COMP_UNDER_SAMPLE
} compensate_event_t;

typedef struct
{
  compensate_event_t    c_evt;              // Type of PCM compensate event
  int64_t               c_sample_skew;      // Number of samples over/under sampled
  int64_t               c_offset;           // Offset between synced and master timer
} compensate_data_s;

void audioInit(void);
void audioStart(void);
void audioDeInit(void);
int16_t* audioGetMicData(void);
void adjustOffset(void);