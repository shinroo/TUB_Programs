#include <stdlib.h>
#include <stdio.h>		//Ein- / Ausgabe
#include <math.h>		//Für die Berechnungen der Ausgabe
#include "blatt11.h"

//In-order-walk for a node
void AVL_in_order_walk_node(AVLNode* node){
	if (node->left != NULL){
		AVL_in_order_walk_node(node->left);
	}
	printf("%d ",node->value);
	if (node->right != NULL){
		AVL_in_order_walk_node(node->right);
	}
}

// Gibt den gesamten AVL Baum in "in-order" Reihenfolge aus.
void AVL_in_order_walk(AVLTree* avlt)
{
	//Hier Code implementieren!
	if ((avlt != NULL) && (avlt->root != NULL)){
		AVL_in_order_walk_node(avlt->root);
		printf("\n");
	}
	else{
		return;
	}

}

// Diese Funktion führt eine Linksrotation auf dem angegebenen Knoten aus.
// Beachtet: Die Höhen der entsprechenden Teilbäume müssen (lokal) angepast werden.
void AVL_rotate_left(AVLTree* avlt, AVLNode* x)
{
	//Hier Code implementieren!
	AVLNode* y = x->right;
	x->right = y->left;
	if (y->left != NULL){
		y->left->parent = x;
	}
	y->parent = x->parent;
	if (x->parent == NULL){
		avlt->root = y;
	}
	else if (x == x->parent->left){
		x->parent->left = y;
	}
	else {
		x->parent->right = y;
	}
	y->left = x;
	x->parent = y;
}

// Diese Funktion führt eine Rechtsrotation auf dem angegebenen Knoten aus.
// Beachtet: Die Höhen der entsprechenden Teilbäume müssen (lokal) angepast werden.
void AVL_rotate_right(AVLTree* avlt, AVLNode* y)
{
	//Hier Code implementieren!
	AVLNode* x = y->left;
	y->left = x->right;
	if (x->right != NULL){
		x->right->parent = y;
	}
	x->parent = y->parent;
	if (y->parent == NULL){
		avlt->root = x;
	}
	else if (y == y->parent->right){
		y->parent->right = x;
	}
	else {
		y->parent->left = x;
	}
	x->right = y;
	y->parent = x;
}

//checks if a given node is null, if it is return 0 else return height
int NULLCheck (AVLNode* node){
	if (node == NULL){
		return 0;
	}
	else{
		return node->height;
	}
}

//Balanciere den Teilbaum unterhalb von node.
void AVL_balance(AVLTree* avlt, AVLNode* node)
{
	if(node == NULL){
		return;
	}

	int lh,rh;

	lh = NULLCheck(node->left);
	rh = NULLCheck(node->right);

	if (lh > rh + 1){
		int llh, lrh;

		llh = NULLCheck(node->left->left);
		lrh = NULLCheck(node->left->right);

		if (llh < lrh){
			AVL_rotate_left(avlt,node->left);
		}
		AVL_rotate_right(avlt,node);
	}
	else if (rh > lh + 1){
		int rrh, rlh;

		rrh = NULLCheck(node->right->right);
		rlh = NULLCheck(node->right->left);

		if (rrh < rlh){
			AVL_rotate_right(avlt,node->right);
		}
		AVL_rotate_left(avlt,node);
	}
}

//function to adapt the heights of a given node and its children
void AVL_heightify(AVLNode* node){
	if(node->left != NULL){
		AVL_heightify(node->left);
	}
	if(node->right != NULL){
		AVL_heightify(node->right);
	}

	if(node->left == NULL && node->right == NULL){
		node->height = 1;
	}
	else{
		//find the max between the two children of the node
		int h1,h2;
		h1 = NULLCheck(node->left);
		h2 = NULLCheck(node->right);
		if (h1 > h2){
			node->height = h1 + 1;
		}
		else{
			node->height = h2 + 1;
		}
	}
}

//function to insert a new node into the AVL tree
void AVL_insert_node(AVLTree* avlt, AVLNode* t, AVLNode* x){
	if (t == NULL){
		if (x->value < x->parent->value){
			x->parent->left = x;
		}
		else{
			x->parent->right = x;
		}
	}
	else if (x->value < t->value){
		x->parent = t;
		AVL_insert_node(avlt,t->left,x);
	}
	else if (x->value > t->value){
		x->parent = t;
		AVL_insert_node(avlt,t->right,x);
	}
	else{
		printf("Schlüssel schon vorhanden\n");
	}

	//adjust height of tree
	AVL_heightify(avlt->root);
	AVL_balance(avlt,t);
	AVL_heightify(avlt->root);
}
// Fügt der Wert value in den Baum ein.
// Die Implementierung muss sicherstellen, dass der Baum nach dem Einfügen
// immer noch balanciert ist!
void AVL_insert_value(AVLTree* avlt, int value)
{
	//create new node
	AVLNode* x = (AVLNode*) malloc (sizeof(AVLNode));
	x->left = NULL;
	x->right = NULL;
	x->parent = NULL;
	x->height = 0;
	x->value = value;

	//insert new node
	if (avlt->root != NULL){
		AVL_insert_node(avlt,avlt->root,x);
	}
	else{
		avlt->root = x;
	}

	avlt->numberOfNodes++;
}

//function which uses post-order to free a node and all its children
void AVL_remove_all_nodes(AVLNode* node){
	if (node->left != NULL){
		AVL_remove_all_nodes(node->left);
	}
	if (node->right != NULL){
		AVL_remove_all_nodes(node->right);
	}
	free(node);
}

// Löscht den gesamten AVL-Baum und gibt den Speicher aller Knoten frei.
void AVL_remove_all_elements(AVLTree* avlt)
{
	//Hier Code implementieren!
	if (avlt != NULL){
		AVL_remove_all_nodes(avlt->root);
	}
	else{
		free(avlt);
	}

}
