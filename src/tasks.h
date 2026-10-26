#ifndef TASKS_H
#define TASKS_H

/* Boot/init task; after initialization it becomes the network supervisor. */
void obc_init_task(void *argument);

#endif /* TASKS_H */
