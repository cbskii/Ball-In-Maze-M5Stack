/* 
 * Basic button driver. Currently only supports presses and works fairly well,
 * but ultimately needs debounce support.
 *
 * TODO rewrite using timer interrupt and debouncing algorithm for multiple
 *      buttons mentioned in http://www.ganssle.com/debouncing-pt2.htm
 */

#ifndef BUTTONS_H
#define BUTTONS_H

/**
 * @brief Configure M5Stack buttons and start button task
 *        Note: requires gpio isr services to be installed first
 */
void buttons_init(void);

/**
 * @brief Unconfigures buttons and stops button task
 */ 
void buttons_exit(void);

#endif /* BUTTONS_H */
