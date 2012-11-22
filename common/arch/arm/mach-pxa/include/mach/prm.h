/*
 * arch/arm/mach-pxa/include/mach/prm.h
 *
 * Copyright (C) 2006, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PRM_H
#define __PRM_H

#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <mach/pmu.h>
#include <mach/pxa95x_dvfm.h>

#define MAX_GROUPS	2
#define MAX_CLIENTS	16

typedef enum {
	/* tag the loweset priority */
	PRI_LOWEST = 0,
	/*define the possible priorities here */
	PRI_IPMC = PRI_LOWEST,
	PRI_PROFILER,
	PRI_VTUNE,
	/*tag the highest priority */
	MAX_PRIORITIES,
	PRI_HIGHEST = MAX_PRIORITIES - 1,
} prm_priority;

struct prm_group;
struct prm_resource;
struct prm_resource_state;

typedef enum {
	PRM_RES_APPROPRIATED,
	PRM_RES_READY,
} prm_event;

typedef enum {
	PRM_CCNT = 0,
	PRM_PMN0,
	PRM_PMN1,
	PRM_PMN2,
	PRM_PMN3,
	PRM_VCC0,
	PRM_VCC1,
	PRM_IDLE_PROFILER,
	PRM_COP,
	RESOURCE_NUM,
} prm_resource_id;

typedef void (*clientcallback) (prm_event, unsigned int, void *);

/* The gourp includes a set of resources. If one of the set of resources is
 * appropriated, the other resources will not available for access. But the
 * resources are still allocated by the client. So the group is defined as
 * a set of resources that all can be accessed or all can not be accessed.
 */
struct prm_group {
	unsigned int id;
	/* appropriated resources count */
	unsigned int appropriated_cnt;
	/* total resources count in the group */
	unsigned int member_cnt;
	/* list for all the resources in the group */
	struct list_head resources;
	struct proc_dir_entry *dir;
};

struct prm_client {
	/* client id */
	unsigned int id;
	/* process id for the client */
	unsigned int pid;
	/* priority for the client.(LOW or HIGH) */
	prm_priority priority;
	/* name of the client */
	char *name;
	/* How many groups in the client */
	unsigned int group_cnt;
	/* support MAXGROUP groups, some may be NULL */
	struct prm_group *groups[MAX_GROUPS];
	void *client_data;
	/* notifier for resource appropriate and ready */
	clientcallback notify;
	irq_handler_t handler;
	void *dev_id;
	struct proc_dir_entry *dir;
};

struct prm_resource_state {
	/* which client allocate the resources. In every priority,
	 * there can be only one client allocate the resource
	 */
	struct prm_client *allocate;
	/* which group it belongs to */
	struct prm_group *group;
	int active;
	struct prm_resource *resource;
	/* used by prm_group->resources for link the resources into the group */
	struct list_head entry;
	struct proc_dir_entry *dir;
};

struct prm_resource {
	struct prm_client *access;	/* Only one client can access it */
	prm_resource_id id;
	struct prm_resource_state priority[MAX_PRIORITIES];
	struct proc_dir_entry *dir;
};

int prm_open_session(prm_priority, char *, clientcallback, void *);
int prm_close_session(unsigned int);
int prm_allocate_resource(unsigned int, prm_resource_id, unsigned int);
int prm_free_resources(unsigned int, unsigned int);
int prm_commit_resources(unsigned int, unsigned int);
int pmu_read_register(unsigned int, int, unsigned int *);
int pmu_write_register(unsigned int, int, unsigned int);
int pmu_set_event(unsigned int, unsigned int, int *, int);
int pmu_enable_event_counting(unsigned int);
int pmu_disable_event_counting(unsigned int);
int pmu_enable_event_interrupt(unsigned int, int);
int pmu_disable_event_interrupt(unsigned int, int);
int pmu_register_isr(unsigned int, irq_handler_t, void *);
int pmu_unregister_isr(unsigned int);
int cop_get_num_of_cops(void);
int cop_get_cop(unsigned int, unsigned int, struct pxa95x_fv_info *);
int cop_set_cop(unsigned int, unsigned int, int mode);
int cop_get_def_cop(unsigned int, unsigned int *, struct pxa95x_fv_info *);
int cop_set_def_cop(unsigned int);
int cop_get_cur_cop(unsigned int, unsigned int *, struct pxa95x_fv_info *);

#endif
