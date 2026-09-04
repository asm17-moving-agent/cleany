#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

void unityOutputStart(unsigned long baudrate);
void unityOutputChar(unsigned int character);
void unityOutputFlush(void);
void unityOutputComplete(void);

#define UNITY_OUTPUT_START() unityOutputStart(115200UL)
#define UNITY_OUTPUT_CHAR(character) unityOutputChar(character)
#define UNITY_OUTPUT_FLUSH() unityOutputFlush()
#define UNITY_OUTPUT_COMPLETE() unityOutputComplete()

#ifdef __cplusplus
}
#endif

#endif
