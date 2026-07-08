///
/// @file   msg_queue.h
/// @brief  Generic thread-safe FIFO used as each worker thread's inbox.
///
/// The producer hands ownership of a malloc'd item to the queue; the consumer
/// frees it after handling. Every sensor worker thread and the use-case thread
/// own one of these as their blocking mailbox, which is how the dispatcher
/// decouples itself from the (potentially blocking) sensor logic.
///
#ifndef MSG_QUEUE_H
#define MSG_QUEUE_H

#include <pthread.h>

/// @brief Singly-linked list node holding one queued item.
typedef struct MSG_NODE
{
    void *item;             ///< Opaque, caller-owned payload pointer.
    struct MSG_NODE *next;  ///< Next node, or NULL at the tail.
} MSG_NODE_T;

/// @brief Thread-safe FIFO queue of void* items (mutex + condition variable).
typedef struct
{
    MSG_NODE_T *head;       ///< Oldest node (dequeued next).
    MSG_NODE_T *tail;       ///< Newest node (most recently pushed).
    int count;              ///< Number of items currently queued.
    pthread_mutex_t mutex;  ///< Guards all queue state.
    pthread_cond_t cond;    ///< Signalled when an item is pushed.
} MSG_QUEUE_T;

///
/// @brief  Initialize an empty queue and its mutex/condition variable.
/// @param  queue_  Queue to initialize. Must not be NULL.
/// @return None.
///
void MsgQueue_Init( MSG_QUEUE_T *queue_ );

///
/// @brief  Append an item to the tail and wake one waiting consumer.
///
/// Ownership of @p item_ transfers to the queue; the consumer that pops it is
/// responsible for freeing it. Safe to call from any thread.
///
/// @param  queue_  Target queue.
/// @param  item_   Malloc'd payload pointer to enqueue.
/// @return None.
///
void MsgQueue_Push( MSG_QUEUE_T *queue_, void *item_ );

///
/// @brief  Remove and return the head item, blocking until one is available.
/// @param  queue_  Source queue.
/// @return The dequeued item pointer (caller must free it).
///
void *MsgQueue_Pop( MSG_QUEUE_T *queue_ );

#endif // MSG_QUEUE_H
