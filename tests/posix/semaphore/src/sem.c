/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <ztest.h>
#include <misc/printk.h>

#define STACK_SIZE 1024

sem_t sema;

static K_THREAD_STACK_DEFINE(stack, STACK_SIZE);

static void *foo_func(void *p1)
{
	printk("Child thread running\n");
	zassert_false(sem_post(&sema), "sem_post failed\n");
	return NULL;
}

static void test_sema(void)
{
	pthread_t newthread;
	pthread_attr_t attr;
	struct sched_param schedparam;
	int schedpolicy = SCHED_FIFO;
	int val;

	schedparam.priority = 1;

	pthread_attr_init(&attr);
	pthread_attr_setstack(&attr, &stack, STACK_SIZE);
	pthread_attr_setschedpolicy(&attr, schedpolicy);
	pthread_attr_setschedparam(&attr, &schedparam);

	zassert_equal(sem_init(&sema, 0, (CONFIG_SEM_VALUE_MAX + 1)), -1,
		      "value larger than %d\n", CONFIG_SEM_VALUE_MAX);
	zassert_equal(errno, EINVAL, NULL);

	zassert_false(sem_init(&sema, 0, 0), "sem_init failed\n");

	zassert_equal(sem_getvalue(&sema, &val), 0, NULL);
	zassert_equal(val, 0, NULL);

	pthread_create(&newthread, &attr, foo_func, NULL);
	zassert_false(sem_wait(&sema), "sem_wait failed\n");

	printk("Parent thread unlocked\n");
	zassert_false(sem_destroy(&sema), "sema is not destroyed\n");
}

void test_main(void)
{
	ztest_test_suite(test_sem, ztest_unit_test(test_sema));
	ztest_run_test_suite(test_sem);
}
