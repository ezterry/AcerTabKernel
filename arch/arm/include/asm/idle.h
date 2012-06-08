#ifndef _ASM_ARM_IDLE_H
#define ASM_ARM_IDLE_H

#define IDLE_START 1
#define IDLE_END   2
#define IDLE_RELAX 3

struct notifier_block;
void idle_notifier_register(struct notifier_block *n);
void idle_notifier_unregister(struct notifier_block *n);

#endif
