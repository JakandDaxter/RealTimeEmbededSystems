#include "Stack.h"

struct StackNode
{
	uint8_t command;
	struct StackNode* next;
};

struct StackNode* NewNode(uint8_t command)
{
	struct StackNode* node = (struct StackNode*)malloc(sizeof(struct StackNode));
	node->command = command;
	node->next = NULL;
	return node;
}

int isEmpty(struct StackNode* root)
{
	return !root;
}

void push(struct StackNode** root, uint8_t command)
{
	struct StackNode* node = NewNode(command);
	node->next = *root;
	*root = node;
}

uint8_t pop(struct StackNode** root)
{
	if(isEmpty(*root))
	{
		return 0;
	}
	struct StackNode* temp = *root;
	uint8_t popped = temp->command;
	*root = (*root)->next;
	free(temp);
	return popped;
}

uint8_t peek(struct StackNode* root)
{
	if(isEmpty(root))return 0;
	return root->command;
}

