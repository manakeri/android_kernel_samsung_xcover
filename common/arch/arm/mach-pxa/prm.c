/*
 * Monahans Profiler Resource Manager
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <linux/proc_fs.h>
#include <linux/rwsem.h>
#include <linux/interrupt.h>
#include <mach/prm.h>
#include <mach/pmu.h>
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>

/*#define DEBUG
 */

#ifdef DEBUG
#define DPRINTK(fmt, args...)	\
	do { printk(KERN_DEBUG "%s: " fmt, __func__ , ## args); } while (0)
#else
#define DPRINTK(fmt, args...)	do {} while (0)
#endif

#define ASSERT_PRIORITY(pri) \
	if ((pri) < PRI_LOWEST || (pri) > PRI_HIGHEST) \
		return -EINVAL;
#define ASSERT_CLIENT_ID(client) \
	if ((client) >= MAX_CLIENTS) \
		return -EINVAL;
#define ASSERT_RESOURCE_ID(resource) \
	if ((resource) < 0 || (resource) >= RESOURCE_NUM) \
		return -EINVAL;
#define ASSERT_GROUP_ID(group) \
	if ((group) >= MAX_GROUPS) \
		return -EINVAL;

#define IS_PRM_RESOURCE(reg)	((reg) >= PMU_CCNT && (reg) <= PMU_PMN3)
#define PMU_PRM(reg)		(reg - 1)

#define IS_HIGHER_PRIORITY(h1, h2)	((h1) > (h2))
#define for_each_lower_priority(index, pri)	\
	for (index = pri - 1; index >= PRI_LOWEST; index = index - 1)

#define STATE_UNDEF		0x1
#define STATE_ACTIVE		0x2
#define STATE_APPROPRIATED	0x3

static struct prm_resource prm_resources[RESOURCE_NUM];
static struct prm_client *prm_clients[MAX_CLIENTS];
static struct prm_client *prm_pmu_client;

struct rw_semaphore prm_sem;

#ifdef DEBUG
static struct proc_dir_entry *clients_root;
static struct proc_dir_entry *resources_root;
static struct proc_dir_entry *prm_root;

#define proc_dump_end(len, page, start, off, count, eof) \
do {						\
	if (len <= off + count)			\
		*eof = 1;			\
	*start = page + off;			\
	len -= off;				\
	if (len > count)			\
		len = count;			\
	if (len < 0)				\
		len = 0;			\
} while (0)

static int dump_group(char *page, char **start, off_t off,
		      int count, int *eof, void *data)
{
	char *buf = page;
	int len;
	struct prm_group *group = (struct prm_group *) data;

	buf += sprintf(buf, "address: 0x%x\n member_cnt: %u\n"
		       " appropriated_cnt: %u\n",
		       (unsigned int) group, group->member_cnt,
		       group->appropriated_cnt);
	len = buf - page;
	proc_dump_end(len, page, start, off, count, eof);
	return len;
}

static int dump_resource_state(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
	int client_id = -1, group_id = -1;
	int i, len;
	char *buf = page;
	struct prm_resource_state *state = (struct prm_resource_state *) data;

	if (state->allocate) {
		for (i = 0; i < MAX_CLIENTS; i++) {
			if (prm_clients[i]
			    && prm_clients[i] == state->allocate) {
				client_id = i;
				break;
			}
		}
		for (i = 0; i < MAX_GROUPS; i++) {
			if (state->allocate->groups[i] &&
			    state->allocate->groups[i] == state->group) {
				group_id = i;
				break;
			}
		}
	}
	buf += sprintf(buf, "allocate: 0x%x(%d)\n group: 0x%x(%d)\n"
		       " active: %u\n resource: 0x%x\n",
		       (unsigned int) state->allocate, client_id,
		       (unsigned int) state->group,
		       group_id, state->active,
		       (unsigned int) state->resource);
	len = buf - page;
	proc_dump_end(len, page, start, off, count, eof);
	return len;
}

static int dump_client(char *page, char **start, off_t off,
		       int count, int *eof, void *data)
{
	int i, len;
	char *buf = page;
	struct prm_client *client = (struct prm_client *) data;

	buf += sprintf(buf, "address: 0x%x\n id: %u\n pid: %u\n"
		       " priority: %u\n name: %s\n group_cnt: %u\n",
		       (unsigned int) client, client->id, client->pid,
		       client->priority, client->name, client->group_cnt);
	for (i = 0; i < MAX_GROUPS; i++) {
		if (client->groups[i])
			buf += sprintf(buf, "group%u address: 0x%x\n",
					i, (unsigned int) client->groups[i]);
	}
	len = buf - page;
	proc_dump_end(len, page, start, off, count, eof);
	return len;
}

static int dump_resource(char *page, char **start, off_t off,
			 int count, int *eof, void *data)
{
	int i, client_id = -1, len;
	char *buf = page;
	struct prm_resource *resource = (struct prm_resource *) data;

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (prm_clients[i] && prm_clients[i] == resource->access) {
			client_id = i;
			break;
		}
	}
	buf += sprintf(buf, " address:0x%x\n access: 0x%x(%d)\n id: %u\n",
		       (unsigned int) resource,
		       (unsigned int) resource->access, client_id,
		       resource->id);
	len = buf - page;
	proc_dump_end(len, page, start, off, count, eof);
	return len;
}

static void proc_add_client(struct prm_client *client)
{
	char buf[16];

	sprintf(buf, "client%d", client->id);
	client->dir = proc_mkdir(buf, clients_root);
	create_proc_read_entry("info", 0, client->dir, dump_client,
			       client);
}

static void proc_del_client(struct prm_client *client)
{
	char buf[16];

	remove_proc_entry("info", client->dir);
	sprintf(buf, "client%d", client->id);
	remove_proc_entry(buf, clients_root);
}

static void proc_add_group(struct prm_client *client,
			   struct prm_group *group, unsigned int group_id)
{
	char buf[16];

	sprintf(buf, "group%d", group_id);
	group->dir = proc_mkdir(buf, client->dir);
	create_proc_read_entry("info", 0, group->dir, dump_group, group);
}

static void proc_del_group(struct prm_client *client,
			   struct prm_group *group, unsigned int group_id)
{
	char buf[32];

	remove_proc_entry("info", group->dir);
	sprintf(buf, "group%d", group_id);
	remove_proc_entry(buf, client->dir);
}

static void proc_add_resource(struct prm_resource *resource)
{
	char buf[16];

	sprintf(buf, "resource%d", resource->id);
	resource->dir = proc_mkdir(buf, resources_root);
	create_proc_read_entry("info", 0, resource->dir,
			       dump_resource, resource);
}

static void proc_del_resource(struct prm_resource *resource)
{
	char buf[16];

	remove_proc_entry("info", resource->dir);
	sprintf(buf, "resource%d", resource->id);
	remove_proc_entry(buf, resources_root);
}

static void proc_add_resource_state(struct prm_resource_state *state,
				    unsigned int priority)
{
	char buf[16];

	sprintf(buf, "state%d", priority);
	state->dir = proc_mkdir(buf, state->resource->dir);
	create_proc_read_entry("info", 0, state->dir,
			       dump_resource_state, state);
}

static void proc_del_resource_state(struct prm_resource_state *state,
				    unsigned int priority)
{
	char buf[16];

	remove_proc_entry("info", state->dir);
	sprintf(buf, "state%d", priority);
	remove_proc_entry(buf, state->resource->dir);
}

static void proc_commit_resource(struct prm_resource *resource)
{
	char buf[32];

	remove_proc_entry("access", resource->dir);
	sprintf(buf, "/proc/prm/clients/%s", resource->access->dir->name);
	proc_symlink("access", resource->dir, buf);
}

static void proc_allocate_resource(struct prm_resource_state *state)
{
	char buf[32], path[64];

	remove_proc_entry("allocate", state->dir);

	sprintf(path, "/proc/prm/clients/%s/%s",
		state->allocate->dir->name, state->group->dir->name);
	proc_symlink("group", state->dir, path);
	sprintf(buf, "resource%d_state%d",
		state->resource->id, state->allocate->priority);
	sprintf(path, "/proc/prm/resources/%s/%s",
		state->resource->dir->name, state->dir->name);
	proc_symlink(buf, state->group->dir, path);
}

static void proc_free_resource(struct prm_resource_state *state)
{
	char buf[32];

	sprintf(buf, "resource%d_state%d",
		state->resource->id, state->allocate->priority);
	remove_proc_entry("access", state->resource->dir);
	remove_proc_entry("allocate", state->dir);
	remove_proc_entry("group", state->dir);
	remove_proc_entry(buf, state->group->dir);
}

static void proc_prm_init(void)
{
	prm_root = proc_mkdir("prm", NULL);
	clients_root = proc_mkdir("clients", prm_root);
	resources_root = proc_mkdir("resources", prm_root);
}

static void proc_prm_exit(void)
{
	remove_proc_entry("clients", prm_root);
	remove_proc_entry("resources", prm_root);
	remove_proc_entry("prm", NULL);
}
#else
static void proc_add_client(struct prm_client *client)
{
}

static void proc_del_client(struct prm_client *client)
{
}

static void proc_add_group(struct prm_client *client,
			   struct prm_group *group, unsigned int group_id)
{
}

static void proc_del_group(struct prm_client *client,
			   struct prm_group *group, unsigned int group_id)
{
}

static void proc_add_resource(struct prm_resource *resource)
{
}

static void proc_del_resource(struct prm_resource *resource)
{
}

static void proc_add_resource_state(struct prm_resource_state *state,
				    unsigned int priority)
{
}

static void proc_del_resource_state(struct prm_resource_state *state,
				    unsigned int priority)
{
}

static void proc_commit_resource(struct prm_resource *resource)
{
}

static void proc_allocate_resource(struct prm_resource_state *state)
{
}

static void proc_free_resource(struct prm_resource_state *state)
{
}

static void proc_prm_init(void)
{
}

static void proc_prm_exit(void)
{
}
#endif

/*****************************************************************************/
/*                                                                           */
/*                  Profiler Resource Manager                                */
/*                                                                           */
/*****************************************************************************/

static void clear_state(struct prm_resource_state *state)
{
	state->allocate = NULL;
	state->group = NULL;
	state->active = STATE_UNDEF;
	/* the state has been deleted from the group resource list */
	INIT_LIST_HEAD(&(state->entry));
}

static int group_commited(struct prm_client *client,
			  struct prm_group *group)
{
	struct prm_resource_state *state;
	struct prm_resource *resource;
	struct list_head *pos;

	list_for_each(pos, &(group->resources)) {
		state = list_entry(pos, struct prm_resource_state, entry);
		resource = state->resource;
		if (resource->access != client) {
			return 0;
		}
	}
	return 1;
}

static int try_to_access_group(struct prm_client *client,
			       struct prm_group *group, int set_state)
{
	struct prm_resource_state *state;
	struct prm_resource *resource;
	int ret = 0;
	struct list_head *pos;

	DPRINTK("client <%d> try to access group <%d>, set_state as <%d>\n",
	     (unsigned int) client->id, (unsigned int) group->id,
	     set_state);
	list_for_each(pos, &(group->resources)) {
		state = list_entry(pos, struct prm_resource_state, entry);
		resource = state->resource;
		if (resource->access != NULL && resource->access != client
		    && IS_HIGHER_PRIORITY(resource->access->priority,
					  client->priority)) {
			if (set_state) {
				state->active = STATE_APPROPRIATED;
				group->appropriated_cnt++;
			}
			ret++;
		}
	}
	DPRINTK("try_to_access() return :%d\n", ret);
	return ret;
}

static struct prm_client *get_resource_access(struct prm_resource *resource)
{
	if (resource)
		return resource->access;
	else			/*for access the isr and control regs of PMU */
		return (struct prm_client *) (
			(unsigned long)prm_resources[PRM_CCNT].access &
			(unsigned long)prm_resources[PRM_PMN0].access &
			(unsigned long)prm_resources[PRM_PMN1].access &
			(unsigned long)prm_resources[PRM_PMN2].access &
			(unsigned long)prm_resources[PRM_PMN3].access);
}

static void unload_isr(struct prm_client *client)
{
	if (prm_pmu_client == client
	    || get_resource_access(NULL) == client)
		prm_pmu_client = NULL;
}

static void load_isr(struct prm_client *client)
{
	if (prm_pmu_client != client
	    && get_resource_access(NULL) == client)
		prm_pmu_client = client;
}

/* this function will be invoked with locked */
static int set_resource_access(struct prm_resource *resource,
			       struct prm_client *client)
{
	struct prm_resource_state *state, *owner_state;
	struct prm_group *group, *owner_group;
	struct prm_client *owner;
	int ret = 0;

	if (client == NULL) {
		/* The client will free the committed resources to the
		 * appropriated lower client. And notification will be sent so
		 * as to give the lower priority client a chance to commit
		 * resources if:
		 * 1. all the resources of the lower priority group that
		 *    resource belongs to are committable
		 * 2. lower priority client hasn't committed above group
		 *    resources
		 * Note: the notified client is unnessarily the appropriated
		 * client.
		 */
		int index;

		DPRINTK("client <%d> give up resource <%d>\n",
			(unsigned int) resource->access->id,
			(unsigned int) resource->id);
		unload_isr(resource->access);
		owner = resource->access;
		resource->access = NULL;
		resource->priority[owner->priority].active = STATE_UNDEF;
		for_each_lower_priority(index, owner->priority) {
			state = &(resource->priority[index]);
			DPRINTK("   its state of lower priority <%d> is <%d>\n",
			     index, state->active);
			if (state->active == STATE_APPROPRIATED) {
				DPRINTK("client <%d> return resource <%d>"
					" to client <%d>\n", owner->id,
					resource->id, state->allocate->id);
				group = state->group;
				group->appropriated_cnt--;
				DPRINTK("resource group <%d> of client <%d>"
				     " has <%d> resources appropriated\n",
				     group->id, state->allocate->id,
				     group->appropriated_cnt);
			}
			if (state->group &&
			    state->group->appropriated_cnt == 0 &&
			    state->allocate && !group_commited(state->allocate,
					    state->group)) {
				ret = try_to_access_group(state->allocate,
							state->group, 1);
				if (ret < 0)
					return ret;
				else if (ret == 0) {
					/* state->active = STATE_UNDEF;
					 */
					/* ISR will not reload, because now the
					 * group has not be commited again.
					 * The isr should be loaded when commit
					 */
					if (state->allocate->notify) {
						up_write(&prm_sem);
						DPRINTK("client <%d> notified"
							" with PRM_RES_READY\n",
							(unsigned int) state->
							allocate->id);
						state->allocate->notify(
							PRM_RES_READY,
							state->group->id,
							state->allocate->
								client_data);
						down_write(&prm_sem);
					}
					break;
				}
			}
		}
	} else {
		struct prm_resource *group_resource;
		struct list_head *pos;

		owner = resource->access;

		if (owner == client) {
			DPRINTK("client <%d> commits resource <%d>:"
				" already commited\n",
				(unsigned int) client->id,
				(unsigned int) resource->id);
			return 0;
		}
		if (!owner)
			unload_isr(owner);
		resource->access = client;
		resource->priority[client->priority].active = STATE_ACTIVE;
		load_isr(client);
		if (owner == NULL) {
			DPRINTK("client <%d> commits resource <%d>:"
				" from free list\n",
				(unsigned int) client->id,
				(unsigned int) resource->id);
			return 0;
		} else {
			DPRINTK("client <%d> commits resource <%d>:"
				" from client <%d>\n\n",
				(unsigned int) client->id,
				(unsigned int) resource->id, owner->id);
		}

		owner_state = &(resource->priority[owner->priority]);
		owner_state->active = STATE_APPROPRIATED;
		owner_group = owner_state->group;
		if (owner_group->appropriated_cnt++ == 0) {
			list_for_each(pos, &(owner_group->resources)) {
				state = list_entry(pos,
						   struct prm_resource_state,
						   entry);
				group_resource = state->resource;
				if (group_resource->access == owner)
					ret = set_resource_access
					    (group_resource, NULL);
				if (ret)
					return ret;
			}
			if (owner->notify) {
				up_write(&prm_sem);
				DPRINTK("client <%d> notified with"
					" PRM_RES_APPROPRIATED\n",
					(unsigned int) owner->id);
				owner->notify(PRM_RES_APPROPRIATED,
					      owner_group->id,
					      owner->client_data);
				down_write(&prm_sem);
			}
		}
	}
	return 0;
}

int prm_open_session(prm_priority priority, char *name,
		     clientcallback callback, void *data)
{
	struct prm_client *client;
	unsigned int name_len;
	int i = 0;

	ASSERT_PRIORITY(priority);
	if (!name) {
		return -EINVAL;
	}
	/* protect for read */
	down_read(&prm_sem);
	for (i = 0; i < MAX_CLIENTS; i++) {
		if (prm_clients[i] == NULL)
			break;
	}
	up_read(&prm_sem);

	if (i == MAX_CLIENTS)
		return -ENOENT;

	name_len = strlen(name);
	client = (struct prm_client *)
	    kmalloc(sizeof(struct prm_client) + name_len + 1, GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	memset(client, 0x0, sizeof(struct prm_client));
	client->id = i;
	client->pid = current->pid;
	client->priority = priority;
	client->notify = callback;
	client->client_data = data;
	client->name = (char *) (client + 1);
	strncpy(client->name, name, name_len);
	client->name[name_len] = '\0';

	for (i = 0; i < MAX_GROUPS; i++)
		client->groups[i] = NULL;

	down_write(&prm_sem);
	if (prm_clients[client->id] != NULL) {
		up_write(&prm_sem);
		kfree(client);
		return -ENOENT;
	}
	prm_clients[client->id] = client;
	up_write(&prm_sem);
	proc_add_client(client);

	DPRINTK("client<%d>(%s) open a session with priority <%d>\n",
		client->id, name, priority);

	return client->id;
}
EXPORT_SYMBOL(prm_open_session);

int prm_close_session(unsigned int client_id)
{
	struct prm_client *client;

	ASSERT_CLIENT_ID(client_id);

	down_write(&prm_sem);
	client = prm_clients[client_id];
	/* resources should be freed before close seesion */
	if (client->group_cnt) {
		up_write(&prm_sem);
		return -EPERM;
	}
	prm_clients[client_id] = NULL;
	up_write(&prm_sem);
	proc_del_client(client);
	kfree(client);

	DPRINTK("client<%d> closed its session\n", client_id);

	return 0;
}
EXPORT_SYMBOL(prm_close_session);

/* allocate resource, but can not access it now */
int prm_allocate_resource(unsigned int client_id,
			  prm_resource_id res_id, unsigned int group_id)
{
	struct prm_client *client;
	struct prm_resource *resource;
	struct prm_resource_state *state;
	struct prm_group *group;

	ASSERT_CLIENT_ID(client_id);
	ASSERT_RESOURCE_ID(res_id);
	ASSERT_GROUP_ID(group_id);

	DPRINTK("allocate resource for client <%d> with resource <%d>"
		" for group <%d>\n", client_id, res_id, group_id);
	down_write(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_write(&prm_sem);
		return -EINVAL;
	}
	resource = &(prm_resources[res_id]);
	state = &(resource->priority[client->priority]);

	/* The resource in the client->priority has been reserved */
	if (state->allocate) {
		up_write(&prm_sem);
		return -EPERM;
	} else
		state->allocate = client;
	group = client->groups[group_id];
	up_write(&prm_sem);

	if (group == NULL) {
		group = (struct prm_group *)
		    kmalloc(sizeof(struct prm_group), GFP_KERNEL);
		if (group == NULL)
			return -ENOMEM;

		INIT_LIST_HEAD(&(group->resources));
		group->id = group_id;
		group->appropriated_cnt = 0;
		group->member_cnt = 1;
		proc_add_group(client, group, group_id);

		down_write(&prm_sem);
		if (client->groups[group_id]) {
			up_write(&prm_sem);
			kfree(group);
			down_write(&prm_sem);
			group = client->groups[group_id];
		} else {
			client->groups[group_id] = group;
			client->group_cnt++;
		}
	} else {
		down_write(&prm_sem);
		client->groups[group_id]->member_cnt++;
	}
	list_add(&(state->entry), &(group->resources));
	state->group = group;
	state->active = STATE_UNDEF;
	up_write(&prm_sem);
	proc_allocate_resource(state);

	return 0;
}
EXPORT_SYMBOL(prm_allocate_resource);

int prm_free_resources(unsigned int client_id, unsigned int group_id)
{
	struct prm_client *client;
	struct prm_resource *resource;
	struct prm_group *group;
	struct prm_resource_state *state;
	int ret = -EINVAL;
	struct list_head *pos, *n;

	ASSERT_CLIENT_ID(client_id);
	ASSERT_GROUP_ID(group_id);

	down_write(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_write(&prm_sem);
		return -EINVAL;
	}
	group = client->groups[group_id];
	if (!group) {
		up_write(&prm_sem);
		return -EINVAL;
	}

	list_for_each_safe(pos, n, &(group->resources)) {
		state = list_entry(pos, struct prm_resource_state, entry);
		resource = state->resource;
		if (get_resource_access(resource) == client) {
			ret = set_resource_access(resource, NULL);
			if (ret) {
				up_write(&prm_sem);
				return ret;
			}
		}
#if 0
		else if (state->active == STATE_APPROPRIATED)
			group->appropriated_cnt--;
#endif
		proc_free_resource(state);
		list_del(pos);
		clear_state(state);
	}
	client->group_cnt--;
	client->groups[group_id] = NULL;
	up_write(&prm_sem);
	proc_del_group(client, group, group_id);
	kfree(group);

	return 0;
}
EXPORT_SYMBOL(prm_free_resources);

int prm_commit_resources(unsigned int client_id, unsigned int group_id)
{
	struct prm_client *client;
	struct prm_group *group;
	struct prm_resource_state *state;
	struct prm_resource *resource;
	struct list_head *pos;
	int ret;

	ASSERT_CLIENT_ID(client_id);
	ASSERT_GROUP_ID(group_id);

	DPRINTK("client <%d> commit resource group <%d>\n",
		client_id, group_id);
	down_write(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_write(&prm_sem);
		return -EINVAL;
	}
	group = client->groups[group_id];
	if (!group) {
		up_write(&prm_sem);
		return -EINVAL;
	}

	ret = try_to_access_group(client, group, 0);
	if (ret) {
		up_write(&prm_sem);
		return ret;
	}

	list_for_each(pos, &(group->resources)) {
		state = list_entry(pos, struct prm_resource_state, entry);
		resource = state->resource;
		ret = set_resource_access(resource, client);
		if (ret) {
			up_write(&prm_sem);
			return ret;
		}
		proc_commit_resource(resource);
	}
	up_write(&prm_sem);
	return 0;
}
EXPORT_SYMBOL(prm_commit_resources);

int prm_get_cpuid(void)
{
	int cpu_id;

	asm("mrc p15, 0, %0, c0, c0" : "=r"(cpu_id));
	cpu_id &= 0xfffff000;

	return cpu_id;
}
EXPORT_SYMBOL(prm_get_cpuid);

static irqreturn_t prm_pmu_handler(int irq, void *dev_id)
{
	/*DPRINTK("PMU interrupt generated!\n");
	 */
	if (prm_pmu_client)
		prm_pmu_client->handler(irq, prm_pmu_client->dev_id);
	return IRQ_HANDLED;
}

/*****************************************************************************/
/*                                                                           */
/*                                 PMU    API                                */
/*                                                                           */
/*****************************************************************************/

int pmu_read_register(unsigned int client_id, int reg, unsigned int *pval)
{
	struct prm_resource *resource;
	struct prm_client *client;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	resource = IS_PRM_RESOURCE(reg) ? &(prm_resources[PMU_PRM(reg)]) :
		NULL;
	ret = (get_resource_access(resource) == client);
	up_read(&prm_sem);

	if (ret)
		*pval = pmu_read_reg(reg);
	else
		return -EACCES;

	return 0;
}
EXPORT_SYMBOL(pmu_read_register);

int pmu_write_register(unsigned int client_id, int reg, unsigned int val)
{
	struct prm_resource *resource;
	struct prm_client *client;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	resource = IS_PRM_RESOURCE(reg) ? &(prm_resources[PMU_PRM(reg)]) : NULL;
	ret = (get_resource_access(resource) == client);
	up_read(&prm_sem);

	if (ret)
		pmu_write_reg(reg, val);
	else
		return -EACCES;

	return 0;
}
EXPORT_SYMBOL(pmu_write_register);

int pmu_set_event(unsigned int client_id, unsigned int counter,
		  int *pre_type, int type)
{
	struct prm_client *client;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	ret = (client == get_resource_access(NULL));
	up_read(&prm_sem);

	if (ret) {
		*pre_type = pmu_select_event(counter, type);
		if (*pre_type == PMU_EVENT_INVALIDATE)
			return -EINVAL;
	} else
		return -EACCES;
	return 0;
}
EXPORT_SYMBOL(pmu_set_event);

int pmu_enable_event_counting(unsigned int client_id)
{
	struct prm_client *client;
	unsigned long val;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	ret = (client == get_resource_access(NULL));
	up_read(&prm_sem);

	if (ret) {
		/* enable and reset all counters,
		 * CCNT counts every clock cycle
		 */
		val = 0x07;
		pmu_write_reg(PMU_PMNC, val);
	} else
		return -EACCES;
	return 0;
}
EXPORT_SYMBOL(pmu_enable_event_counting);

int pmu_disable_event_counting(unsigned int client_id)
{
	struct prm_client *client;
	unsigned long val;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	ret = (client == get_resource_access(NULL));
	up_read(&prm_sem);

	if (ret) {
		/* disable all counters */
		val = 0x10;
		pmu_write_reg(PMU_PMNC, val);
	} else
		return -EACCES;
	return 0;
}
EXPORT_SYMBOL(pmu_disable_event_counting);

int pmu_enable_event_interrupt(unsigned int client_id, int reg)
{
	struct prm_client *client;
	unsigned long val;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	if (IS_PRM_RESOURCE(reg)) {
		ret = (get_resource_access(&(prm_resources[PMU_PRM(reg)])) ==
		     client);
		up_read(&prm_sem);
		if (ret) {
			val = pmu_read_reg(PMU_INTEN);
			val |= (0x1 << reg);
			pmu_write_reg(PMU_INTEN, val);
		} else
			return -EACCES;
	} else {
		up_read(&prm_sem);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(pmu_enable_event_interrupt);

int pmu_disable_event_interrupt(unsigned int client_id, int reg)
{
	struct prm_client *client;
	unsigned long val;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	if (IS_PRM_RESOURCE(reg)) {
		ret = (get_resource_access(&(prm_resources[PMU_PRM(reg)])) ==
		     client);
		up_read(&prm_sem);
		if (ret) {
			val = pmu_read_reg(PMU_INTEN);
			val &= ~(0x1 << reg);
			pmu_write_reg(PMU_INTEN, val);
		} else
			return -EACCES;
	} else {
		up_read(&prm_sem);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(pmu_disable_event_interrupt);

int pmu_register_isr(unsigned int client_id,
		     irq_handler_t handler, void *dev_id)
{
	struct prm_client *client;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	client->handler = handler;
	client->dev_id = dev_id;
	load_isr(client);
	up_read(&prm_sem);
	return 0;
}
EXPORT_SYMBOL(pmu_register_isr);

int pmu_unregister_isr(unsigned int client_id)
{
	struct prm_client *client;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	unload_isr(client);
	client->handler = NULL;
	client->dev_id = NULL;
	up_read(&prm_sem);
	return 0;
}
EXPORT_SYMBOL(pmu_unregister_isr);

/*****************************************************************************/
/*                                                                           */
/*                                 COP    API                                */
/*                                                                           */
/*****************************************************************************/

int cop_get_num_of_cops(void)
{
	return dvfm_op_count();
}
EXPORT_SYMBOL(cop_get_num_of_cops);

int cop_get_cop(unsigned int client_id, unsigned int n,
		struct pxa95x_fv_info *param)
{
	struct op_info *info = NULL;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	ret = dvfm_get_opinfo(n, &info);
	if (ret == 0) {
		md2fvinfo(param, (struct dvfm_md_opt *) info->op);
	}
	return ret;
}
EXPORT_SYMBOL(cop_get_cop);

int cop_set_cop(unsigned int client_id, unsigned int n, int mode)
{
	struct prm_resource *resource;
	struct prm_client *client;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	resource = &prm_resources[PRM_COP];
	ret = (get_resource_access(resource) == client);
	up_read(&prm_sem);

	if (ret)
		return dvfm_request_op(n);
	return -EACCES;
}
EXPORT_SYMBOL(cop_set_cop);

int cop_get_def_cop(unsigned int client_id, unsigned int *n,
		    struct pxa95x_fv_info *param)
{
	struct op_info *info = NULL;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	*n = dvfm_get_defop();
	ret = dvfm_get_opinfo(*n, &info);
	if (ret == 0) {
		md2fvinfo(param, (struct dvfm_md_opt *) info->op);
	}
	return ret;
}
EXPORT_SYMBOL(cop_get_def_cop);

int cop_set_def_cop(unsigned int client_id)
{
	struct prm_resource *resource;
	struct prm_client *client;
	unsigned int def_op;
	int ret;

	ASSERT_CLIENT_ID(client_id);

	down_read(&prm_sem);
	client = prm_clients[client_id];
	if (!client) {
		up_read(&prm_sem);
		return -EINVAL;
	}

	resource = &prm_resources[PRM_COP];
	ret = (get_resource_access(resource) == client);
	up_read(&prm_sem);

	def_op = dvfm_get_defop();
	if (ret)
		return dvfm_request_op(def_op);
	return -EACCES;
}
EXPORT_SYMBOL(cop_set_def_cop);

int cop_get_cur_cop(unsigned int client_id, unsigned int *n,
		    struct pxa95x_fv_info *param)
{
	struct op_info *info = NULL;

	ASSERT_CLIENT_ID(client_id);

	*n = dvfm_get_op(&info);
	md2fvinfo(param, (struct dvfm_md_opt *) info->op);

	return 0;
}
EXPORT_SYMBOL(cop_get_cur_cop);

/*****************************************************************************/
/*                                                                           */
/*                                 Module Init/Exit                          */
/*                                                                           */
/*****************************************************************************/

static int __init prm_init(void)
{
	int ret, i, j;

	proc_prm_init();
	init_rwsem(&prm_sem);
	/*prm_sem.debug = 1;
	 */
	for (i = 0; i < RESOURCE_NUM; i++) {
		prm_resources[i].access = NULL;
		prm_resources[i].id = i;
		proc_add_resource(&prm_resources[i]);
		for (j = 0; j < MAX_PRIORITIES; j++) {
			prm_resources[i].priority[j].resource =
			    &prm_resources[i];
			prm_resources[i].priority[j].allocate = NULL;
			prm_resources[i].priority[j].active = STATE_UNDEF;
			INIT_LIST_HEAD(&(prm_resources[i].priority[j].entry));
			proc_add_resource_state(&prm_resources[i].
						priority[j], j);
		}
	}

	for (i = 0; i < MAX_CLIENTS; i++) {
		prm_clients[i] = NULL;
	}
	prm_pmu_client = NULL;

	ret = request_irq(IRQ_PMU, prm_pmu_handler, 0, "PMU", NULL);
	if (ret < 0) {
		DPRINTK("PMU interrupt handler registeration: failed!\n");
		return ret;
	} else {
		DPRINTK("PMU interrupt handler registeration: OK!\n");
	}

	DPRINTK("CPU_ID = 0x%08x\n", prm_get_cpuid());

	return 0;
}

static void __exit prm_exit(void)
{
	int i, j;

	for (i = 0; i < RESOURCE_NUM; i++) {
		for (j = 0; j < MAX_PRIORITIES; j++) {
			proc_del_resource_state(&prm_resources[i].
						priority[j], j);
		}
		proc_del_resource(&prm_resources[i]);
		memset(&(prm_resources[i]), 0x0,
		       sizeof(struct prm_resource));
	}

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (prm_clients[i]) {
			if (prm_clients[i]->group_cnt) {
				for (j = 0; j < MAX_GROUPS; j++) {
					if (prm_clients[i]->groups[j]) {
						proc_del_group(prm_clients[i],
								prm_clients[i]->
								groups[j], j);
						kfree(prm_clients[i]->
								groups[j]);
					}
				}
			}
			proc_del_client(prm_clients[i]);
			kfree(prm_clients[i]);
		}
	}
	prm_pmu_client = NULL;
	free_irq(IRQ_PMU, NULL);
	proc_prm_exit();
}

module_init(prm_init);
module_exit(prm_exit);

MODULE_DESCRIPTION("Performance Resources Management");
MODULE_LICENSE("GPL");
