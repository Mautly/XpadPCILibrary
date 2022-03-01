/*******************************************************
                       xpci_time.h

 Commodity to measure time in program sections.
*******************************************************/
#ifndef XPCI_TIMER
#define XPCI_TIMER

#if defined(__cplusplus)
    extern "C" {
#endif
void xpci_timerStart(int id);
void xpci_timerStop(int id);
#ifdef __cplusplus
}
#endif
#endif
