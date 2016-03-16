#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct _node node;
typedef struct _height_one_tree height_one_tree;

struct _height_one_tree {
	node *root;
};

struct _node {
	node *right;
	node *left;
	int value;
};

void print_tree(height_one_tree *atree) {
	node* root = atree->root;
	if (root == NULL) {
		printf("Leer!\n\n");
		return;
	}
	printf("    %04d\n", root->value);
	printf("   /    \\\n");
	printf("  /      \\\n");
	if (root->left == NULL) {
		printf("NULL");
	} else {
		printf("%04d", root->left->value);
	}
	printf("    ");
	if (root->right == NULL) {
		printf("NULL");
	} else {
		printf("%04d", root->right->value);
	}
	printf("\n\n");
	return;
}

// Nur für Bäume der Höhe der 3
void insert_node(height_one_tree *atree, int value) {
	node *anode = malloc(sizeof(node));		//Speicher für neues Element alloziieren
	anode->value = value;					//Init
	anode->left = NULL;						//Init
	anode->right = NULL;					//Init
	if (atree->root == NULL)				//Fall 1: Randbedingung: Baum ist leer
	{
		atree->root = anode;
		return;								//verlasse Funktion hier
	}
	if (anode->value <= atree->root->value)	//Fall 2: Neues Element <= Wurzel
	{
		atree->root->left = anode;
	}
	else									//Fall 2: Neues Element > Wurzel
	{
		atree->root->right = anode;
	}
}

void free_tree(height_one_tree *atree) {
	if (atree->root != NULL)				//Free nur nötig, wenn Element in Baum
	{
		if (atree->root->left != NULL)		//Linkes Kind nur löschen wenn eines vorhanden
		{
			free(atree->root->left);
		}
		if (atree->root->right != NULL)		//Siehe linkes Kind
		{
			free(atree->root->right);
		}
		free(atree->root);
	}
}

int main() {
	height_one_tree atree;
	atree.root = NULL;

	// Werte 15, 9, 20
	// Speicherzuweisung
	print_tree(&atree);
	insert_node(&atree, 15);
	print_tree(&atree);
	insert_node(&atree, 9);
	print_tree(&atree);
	insert_node(&atree, 20);
	print_tree(&atree);

	// Speicherfreigabe
	free_tree(&atree);
}
