
#include <stdlib.h>
#include "FreeRTOS.h"
 // #include "list.h"    // remplacé ci dessous par son code
struct xLIST;
struct xLIST_ITEM
{   TickType_t xItemValue;			 
	struct xLIST_ITEM *  pxNext;		 
	struct xLIST_ITEM *  pxPrevious;	 
	void * pvOwner;										 
	struct xLIST *  pvContainer;		 
};
typedef struct xLIST_ITEM ListItem_t;					 
struct xMINI_LIST_ITEM
{   TickType_t xItemValue;
	struct xLIST_ITEM *  pxNext;
	struct xLIST_ITEM *  pxPrevious;
};
typedef struct xMINI_LIST_ITEM MiniListItem_t;
typedef struct xLIST
{
	volatile UBaseType_t uxNumberOfItems;
	ListItem_t *  pxIndex;			 
	MiniListItem_t xListEnd;							 
} List_t;
void vListInitialise( List_t * const pxList ) ;
void vListInitialiseItem( ListItem_t * const pxItem ) ;
void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem ) ;
void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem ) ;
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove ) ;

void vListInitialise( List_t * const pxList )
{	pxList->pxIndex = ( ListItem_t * ) &( pxList->xListEnd );			 
	pxList->xListEnd.xItemValue = ( TickType_t ) 0xffffffffUL;
	pxList->xListEnd.pxNext = ( ListItem_t * ) &( pxList->xListEnd );	 
	pxList->xListEnd.pxPrevious = ( ListItem_t * ) &( pxList->xListEnd ); 
	pxList->uxNumberOfItems = ( UBaseType_t ) 0U;
}
void vListInitialiseItem( ListItem_t * const pxItem )
{
	pxItem->pvContainer = 0;
}
void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem )
{
ListItem_t * const pxIndex = pxList->pxIndex;
	;
	;
	pxNewListItem->pxNext = pxIndex;
	pxNewListItem->pxPrevious = pxIndex->pxPrevious;
	;
	pxIndex->pxPrevious->pxNext = pxNewListItem;
	pxIndex->pxPrevious = pxNewListItem;
	pxNewListItem->pvContainer = pxList;
	( pxList->uxNumberOfItems )++;
}
void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem )
{
ListItem_t *pxIterator;
const TickType_t xValueOfInsertion = pxNewListItem->xItemValue;
	if( xValueOfInsertion == ( TickType_t ) 0xffffffffUL )
	{
		pxIterator = pxList->xListEnd.pxPrevious;
	}
	else
	{	for( pxIterator = ( ListItem_t * ) &( pxList->xListEnd ); \
		     pxIterator->pxNext->xItemValue <= xValueOfInsertion; \
		     pxIterator = pxIterator->pxNext )   {}
	}
	pxNewListItem->pxNext = pxIterator->pxNext;
	pxNewListItem->pxNext->pxPrevious = pxNewListItem;
	pxNewListItem->pxPrevious = pxIterator;
	pxIterator->pxNext = pxNewListItem;
	pxNewListItem->pvContainer = pxList;
	( pxList->uxNumberOfItems )++;
}
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove )
{
List_t * const pxList = pxItemToRemove->pvContainer;
	pxItemToRemove->pxNext->pxPrevious = pxItemToRemove->pxPrevious;
	pxItemToRemove->pxPrevious->pxNext = pxItemToRemove->pxNext;
	;
	if( pxList->pxIndex == pxItemToRemove )
	{ pxList->pxIndex = pxItemToRemove->pxPrevious;	}
	
	pxItemToRemove->pvContainer = 0;
	( pxList->uxNumberOfItems )--;
	return pxList->uxNumberOfItems;
}
