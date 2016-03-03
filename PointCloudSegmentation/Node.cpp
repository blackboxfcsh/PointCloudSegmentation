
#include "Node.h"

Node::Node()
{
	// default constructor
	// this is to allow us to create an object without any initialization

}

Node::~Node()
{
	delete outputCloud;
	// default constructor
	// this is to allow us to create an object without any initialization

}

//  This constructor is just to set next pointer of a node and the data contained.
Node::Node(OutputCloud* item, Node* ptrprevious)
{
	this->outputCloud = item;
	this->previous = ptrprevious;
}

/*
//  This methods inserts a node just after the node that the method belongs to 
//  TO-DO: Consider a better implementation
void Node::InsertAfter(Node* p)
{
	// not to lose the rest of the list, we ought to link the rest of the
	// list to the Node* p first
	p->next = this->next;

	// now we should link the previous Node to Node *p , i.e the Node that we are 
	//inserting after,
	this->next = p;
}

// Deletes the node from the list and returns the deleted node
Node* Node::DeleteAfter()
{
	// store the next Node in a temporary Node
	Node* tempNode = next;
	// check if there is a next node
	if (next != NULL)
		next = next->next;

	return tempNode;
}

Node* Node::GetNode(const OutputCloud& item, Node* nextptr)
{
	Node* newnode; // Local ptr for new node
	newnode = new Node(item, nextptr);
	if (newnode == NULL)
	{
		cerr << "Memory allocation failed." << endl;
		exit(1);
	}
	return newnode;
}
*/