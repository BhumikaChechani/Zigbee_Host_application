///
/// @file   msg_queue.c
/// @brief  Implementation of the generic thread-safe FIFO (see msg_queue.h).
///
#include "msg_queue.h"
#include <stdlib.h>

void MsgQueue_Init( MSG_QUEUE_T *queue_ )
{
    queue_->head = NULL;
    queue_->tail = NULL;
    queue_->count = 0;
    pthread_mutex_init( &queue_->mutex, NULL );
    pthread_cond_init( &queue_->cond, NULL );
}

void MsgQueue_Push( MSG_QUEUE_T *queue_, void *item_ )
{
    MSG_NODE_T *node = (MSG_NODE_T *)malloc( sizeof( MSG_NODE_T ) );
    if ( node == NULL )
    {
        return;
    }
    node->item = item_;
    node->next = NULL;

    pthread_mutex_lock( &queue_->mutex );
    if ( queue_->tail != NULL )
    {
        queue_->tail->next = node;
    }
    else
    {
        queue_->head = node;
    }
    queue_->tail = node;
    queue_->count++;
    pthread_cond_signal( &queue_->cond );
    pthread_mutex_unlock( &queue_->mutex );
}

void *MsgQueue_Pop( MSG_QUEUE_T *queue_ )
{
    pthread_mutex_lock( &queue_->mutex );
    while ( queue_->head == NULL )
    {
        pthread_cond_wait( &queue_->cond, &queue_->mutex );
    }
    MSG_NODE_T *node = queue_->head;
    void *item = node->item;
    queue_->head = node->next;
    if ( queue_->head == NULL )
    {
        queue_->tail = NULL;
    }
    queue_->count--;
    pthread_mutex_unlock( &queue_->mutex );

    free( node );
    return item;
}
