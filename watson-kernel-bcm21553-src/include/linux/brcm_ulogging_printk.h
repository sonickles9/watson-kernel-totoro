/* ----------------------------------------------------------- */
/* Unified Logging */

extern void BCMLOG_LogString(const char *inLogString,
				unsigned short inSender);

int bcmlog_mtt_on;
unsigned short bcmlog_log_ulogging_id;

/* ------------------------------------------------------------ */

int brcm_retrive_early_printk(void);
