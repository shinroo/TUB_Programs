#include "blatt05.h"

// Fügt einen Knoten mit der Telefonnummer phone und dem Namen name in den
// Binären Suchbaum bst ein.  Für den Suchbaum soll die Eigenschaft gelten, dass
// alle linken Kinder einen Wert kleiner gleich (<=) und alle rechten Kinder
// einen Wert größer (>) haben.
void bst_insert_node(bstree* bst, int phone, char *name) {
	bst_node* tempnode = bst->root;
	bst_node* parent = NULL;

	if (tempnode == NULL){
		//tree is empty
		bst_node* new = (bst_node*) malloc (sizeof(bst_node));
		new->right = NULL;
		new->left = NULL;
		new->parent = NULL;
		new->phone = phone;

		//add name to new node
		char* temp = (char*) malloc (MAX_STR*sizeof(char)+1);
		strncpy(temp,name,MAX_STR);
		temp[MAX_STR]='\0';
		new->name = malloc(MAX_STR*sizeof(char)+1);
		strncpy(new->name,temp,MAX_STR+1);
		free(temp);
		bst->root = new;
		bst->count++;
	}
	else {
		//if node exists show error
		if (find_node(bst,phone)){
			printf("that phone number exists already\n");
		}
		else{
			//navigate to where node should be added
			while (tempnode != NULL) {
				parent = tempnode;
				if (tempnode->phone < phone) {
					tempnode = tempnode->right;
				}
				else {
					tempnode = tempnode->left;
				}
			}

			//add new node
			bst_node* new = (bst_node*) malloc (sizeof(bst_node));
			new->right = NULL;
			new->left = NULL;
			new->parent = parent;
			new->phone = phone;

			//add name to node
			char* temp = (char*) malloc (MAX_STR*sizeof(char)+1);
			strncpy(temp,name,MAX_STR);
			temp[MAX_STR]='\0';
			new->name = malloc(MAX_STR*sizeof(char)+1);
			strncpy(new->name,temp,MAX_STR+1);
			free(temp);
			bst->count++;

			//assign parent right/left to the node
			if (parent->phone > phone) {
				parent->left = new;
			}
			else {
				parent->right = new;
			}

		}
	}
}

// Diese Funktion liefert einen Zeiger auf einen Knoten im Baum
// mit dem Wert phone zurück, falls dieser existiert. Ansonsten wird
// NULL zurückgegeben.
bst_node* find_node(bstree* bst, int phone) {

	bst_node* out;
	out = bst->root;

	//navigate through tree until node found or end of tree
	while (out != NULL){
		if (out->phone == phone){
			break;
		}
		else if (out->phone > phone){
			out = out->left;
		}
		else{
			out = out->right;
		}
	}

	return out;

}


// Forward-declaration von der Funktion bst_in_order_walk_node
void bst_in_order_walk_node(bst_node* node);

// Gibt den Unterbaum von node in "in-order" Reihenfolge aus
void bst_in_order_walk_node(bst_node* node) {
	if(node!=NULL){
		if(node->left != NULL)
			bst_in_order_walk_node(node->left);
		print_node(node);
		if (node->right != NULL)
			bst_in_order_walk_node(node->right);
	}
}

// Gibt den gesamten Baum bst in "in-order" Reihenfolge aus. Die Ausgabe
// dieser Funktion muss aufsteigend soriert sein.
void bst_in_order_walk(bstree* bst) {
	if (bst != NULL) {
		bst_in_order_walk_node(bst->root);
	}
}

// Löscht den Teilbaum unterhalb des Knotens node rekursiv durch
// "post-order" Traversierung, d.h. zurerst wird der linke und dann
// der rechte Teilbaum gelöscht. Anschließend wird der übergebene Knoten
// gelöscht.
void bst_free_subtree(bst_node* node){
	if(node->left != NULL)
		bst_free_subtree(node->left);
	if(node->right != NULL)
		bst_free_subtree(node->right);
	free(node->name);
	free(node);
	return;
}

// Löscht den gesamten Baum bst und gibt den entsprechenden Speicher frei.
void bst_free_tree(bstree* bst) {
	if(bst != NULL && bst->root != NULL) {
		bst_free_subtree(bst->root);
		bst->root = NULL;
	}
}
