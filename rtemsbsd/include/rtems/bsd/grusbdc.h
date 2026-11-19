#ifndef _RTEMS_BSD_GRUSBDC_H_
#define _RTEMS_BSD_GRUSBDC_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Select the global bulk echo template on a system with one USB device
 * controller. Call before rtems_bsd_initialize().
 */
void rtems_bsd_grusbdc_use_echo_template(void);

#ifdef __cplusplus
}
#endif

#endif /* _RTEMS_BSD_GRUSBDC_H_ */
