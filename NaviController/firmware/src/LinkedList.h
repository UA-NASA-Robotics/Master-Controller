/* 
 * File:     LinkedList.h
 * Author:   Philip Klostermann 
 */

/*
 * Node Type: LL_node_t struct
 */
#ifndef LINKEDLIST_H
#define LINKEDLIST_H
typedef struct node_t {
  void *data;
  struct node_t *prev;
  struct node_t *next;
} LL_node_t;

/*
 * Generic Linked List Type: LL_t
 */
typedef struct {
  unsigned int size;
  LL_node_t *first;
  LL_node_t *last;
} LL_t;


/*  create new list */
LL_t *LL_init();

/*  get/find functions */
void *LL_get(LL_t *, int);
void *LL_first(LL_t *);
void *LL_last(LL_t *);

/*  add functions */
int LL_add(LL_t *, void *, int);
void *LL_set(LL_t *, void *, int);
int LL_push(LL_t *, void *);
int LL_pushBack(LL_t *, void *);

/*  remove functions */
void *LL_remove(LL_t *, int);
void *LL_pop(LL_t *);
void *LL_popBack(LL_t *);

/*  iterate functions */
void LL_each(LL_t *, void (*f)(void *));
void LL_eachReverse(LL_t *, void (*f)(void *));

/*  destructive functions */
void LL_clear(LL_t *);
void LL_destroy(LL_t *);
#endif