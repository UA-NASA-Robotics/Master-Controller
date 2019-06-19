/*
 * File:      LL.c
 * Author:    Philip Klostermann
 * Source:    https://github.com/philbot9/generic-linked-list/blob/master/src
 */

#include <stdio.h>
#include <stdlib.h>
#include "LinkedList.h"

#define C_OK 0
#define C_NOK -1

static LL_node_t *LL_findNode(LL_t *, int);
static LL_node_t *LL_initNode(void *);

/*  
 * Initialize a new list
 * returns:   pointer to new list
 */
LL_t *LL_init() 
{
  LL_t *list = (LL_t *) malloc(sizeof(LL_t));
  list->size = 0;
  list->first = NULL;
  list->last = NULL;
  return list;
}

/*
 * Helper function:
 * Initialize a new node
 * in:        pointer to data
 * returns:   pointer to new node
 */
static LL_node_t *LL_initNode(void *data) 
{
  LL_node_t *node = (LL_node_t *) malloc(sizeof(LL_node_t));
  node->data = data;
  node->prev = NULL;
  node->next = NULL;
  return node;
}

/*
 * Get element at arbitrary position 
 * in:        pointer to list
 * in:        position
 * returns:   void pointer to data / NULL on failure
 */
void *LL_get(LL_t *list, int pos) 
{
  LL_node_t *node = LL_findNode(list, pos);
  if(node != NULL) 
    return node->data;
  else
    return NULL;
}

/*
 * return the first element in the list
 * in:        list pointer
 * returns:   data of first element / NULL on failure
 */
void *LL_first(LL_t *list)
{
  if(list->first == NULL)
    return NULL;

  return list->first->data;
}

/*
 * return the last element in the list
 * in:        list pointer
 * returns:   data of last element / NULL on failure
 */
void *LL_last(LL_t *list)
{
  if(list->last == NULL)
    return NULL;

  return list->last->data;
}

/*
 * Helper function:
 * Find node at a given position
 * in:        pointer to list
 * in:        position
 * returns:   pointer to Node / NULL on failure
 */
static LL_node_t *LL_findNode(LL_t *list, int pos) 
{
  if(pos > list->size || pos < 0)
    return NULL;  

  LL_node_t *currNode;
  int currPos;
  int reverse; 
   
  /* decide where to start iterating from (font or back of the list) */
  if(pos > ((list->size-1) / 2)) {
    reverse  = 1;
    currPos  = list->size - 1;
    currNode = list->last;
  } else {
    reverse  = 0;
    currPos  = 0;
    currNode = list->first;
  }  
  
  while(currNode != NULL) {
    if(currPos == pos)
      break;

    currNode = (reverse ? (currNode->prev) : (currNode->next));
    currPos  = (reverse ? (currPos-1) : (currPos+1));
  }
  return currNode;
}

/*
 * Add new element add an arbitray position
 * in:        pointer to list
 * in:        pointer to data
 * in:        position
 * returns:   0 on success, -1 on failure
 */
int LL_add(LL_t *list, void *data, int pos) 
{
  if(pos > list->size || pos < 0)
    return C_NOK;

  LL_node_t *newNode;
  LL_node_t *currNode;

  /* Create the new node */
  newNode = LL_initNode(data);

  /* if list is empty */
  if(list->size == 0) {
    list->first = newNode;
    list->last = newNode;

    list->size++;
    return C_OK;
  }
  
  /* if list is not empty */
  currNode = LL_findNode(list, pos);

  /* adding at the front or in the middle */
  if(currNode != NULL) {
    newNode->prev = currNode->prev;
    newNode->next = currNode;  
    
    if(currNode->prev == NULL) 
      list->first = newNode;
    else
      currNode->prev->next = newNode;

    currNode->prev = newNode;
  } else { /* adding at the end */
    list->last->next = newNode;
    newNode->prev = list->last;
    list->last = newNode;    
  }
  list->size++;
  return C_OK;
}


/*
 * replace data of node at pos with new data
 * in:        pointer to list
 * in:        pointer to data
 * in:        position in list
 * returns:   data previously stored at pos
 */
void *LL_set(LL_t *list, void *data, int pos) 
{
  LL_node_t *currNode = LL_findNode(list, pos);

  if(currNode == NULL)
    return NULL;
  
  void *oldData = currNode->data;
  currNode->data = data;

  return oldData;
}

/*
 * add element to front of list
 * in:        pointer to list
 * in:        pointer to data
 * returns:   0 on success, -1 on failure
 */
int LL_push(LL_t *list, void *data) 
{
  LL_node_t *newNode = LL_initNode(data); 

  /* if list is empty */
  if(list->size == 0) {
    list->last = newNode;
  } else {
    /* if there is at least one element */
    list->first->prev = newNode;
    newNode->next = list->first;
  }
  list->first = newNode;
  list->size++;
  return C_OK;
}

/*
 * add element to end of list
 * in:        pointer to list
 * in:        pointer to data
 * returns:   0 on success, -1 on failure
 */
int LL_pushBack(LL_t *list, void *data) 
{
  /* initialize new node */
  LL_node_t *newNode = LL_initNode(data);

  /* if list is empty */
  if(list->size == 0) {
    list->first = newNode;
  } else {
    /* if there is at least one element */
    list->last->next = newNode;
    newNode->prev = list->last;
  }
  list->last = newNode;
  list->size++;
  return C_OK;
}

/*
 * remove from an arbitrary position
 * in:        pointer to list
 * in:        pointer to data
 * returns:   0 on success, -1 on failure
 */
void *LL_remove(LL_t *list, int pos) 
{
  LL_node_t *currNode = LL_findNode(list, pos);
  void *data = NULL;

  if(currNode == NULL)
    return NULL;

  data = currNode->data;

  if(currNode->prev == NULL) 
    list->first = currNode->next;
  else 
    currNode->prev->next = currNode->next;
  
  if(currNode->next == NULL)
    list->last = currNode->prev; 
  else
    currNode->next->prev = currNode->prev;

  list->size--;
  free(currNode);
  return data;
}

/*
 * remove the head of the list and return its value
 * in:        pointer to list
 * returns:   pointer to data of first node/NULL if empty
 */
void *LL_pop(LL_t *list) 
{
  if(!list)
    return NULL;

  LL_node_t *node = list->first;
  if(node == NULL)
    return NULL;

  void *data = node->data;

  if(LL_remove(list, 0) == NULL)
    return NULL;

  return data;
}

/*
 * remove the tail of the list and return its value
 * in:        pointer to list
 * returns:   pointer to data of last node/NULL if empty
 */ 
void *LL_popBack(LL_t *list) 
{
  LL_node_t *node = list->last;
  if(node == NULL)
    return NULL;

  void *data = node->data;

  if(LL_remove(list, (list->size-1)) == NULL)
    return NULL;

  return data;
}

/*
 * iterates over the entire list from the beginning and 
 * calls the specified function with with each element.
 * in:        pointer to list
 * in:        pointer to function
 *            ** function must be of return type void and
 *            ** take void pointer as parameter
 */
void LL_each(LL_t *list, void (*f)(void *)) 
{
  LL_node_t *currNode = list->first;

  while(currNode != NULL) {
    (*f)(currNode->data);
    currNode = currNode->next;
  }
}

/*
 * iterates over the entire list from the end and
 * calls the specified function with each element.
 * in:        pointer to list
 * in:        pointer to function
 *            ** function must be of return type void and
 *            ** take void pointer as parameter
 */
void LL_eachReverse(LL_t *list, void (*f)(void *)) 
{
  LL_node_t *currNode = list->last;

  while(currNode != NULL) {
    (*f)(currNode->data);
    currNode = currNode->prev;
  }
}

/*
 * destroys the list and allocates a new (empty)
 * in its memory location
 * in:        pointer to list
 */
void LL_clear(LL_t *list) 
{
  LL_node_t *currNode = list->first;
  LL_node_t *nextNode;

  while(currNode != NULL) {
    nextNode = currNode->next;
    free(currNode);
    currNode = nextNode;
  }

  list->first = NULL;
  list->last = NULL;
  list->size = 0;
}

/*
 * destroys a list and frees all list related memory
 * Does not touch the data stored at the nodes!
 * in:        pointer to list
 */
void LL_destroy(LL_t *list) 
{
  LL_node_t *currNode = list->first;
  LL_node_t *nextNode;
  
  while(currNode != NULL) {
    nextNode = currNode->next;
    free(currNode);
    currNode = nextNode;  
  }
  free(list);
} 
