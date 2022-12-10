#ifndef SYS_SETUP_H
#define SYS_SETUP_H

struct k_timer my_timer;
extern void my_timer_handler(struct k_timer*);
void my_work_handler(struct k_work*);

#endif SYS_SETUP_H