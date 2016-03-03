
#include "OutputCloud.h"
#include <iostream>

using namespace std;

class Node{

	public:

		Node();
		~Node();
		Node(OutputCloud* item, Node* ptrprevious = NULL);
		
		void setOutuputCloud(OutputCloud* oc) { outputCloud = oc; }
		void setPreviousNode(Node* node) { previous = node; }
		
		Node* getPreviousNode() { return previous; }
		OutputCloud* getOutputCloud() { return outputCloud; }

		//void InsertAfter(Node* p);
		//Node* DeleteAfter();
		//Node* GetNode(const OutputCloud& item, Node* nextptr = NULL);
	
	private:

		Node* previous;
		OutputCloud* outputCloud;
};