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

// Nur für Bäume der Höhe der 1
void insert_node(height_one_tree *atree, int value) {
	node *blub = malloc(sizeof(node));
	blub->left = NULL;
	blub->right = NULL;
	blub->value = value;

	if(atree->root == NULL){
		atree->root = blub;
		return;
	}
	if(atree->root->value > value){
		atree->root->left = blub;
	} else {
		atree->root->right = blub;
	}
    
}

//Löscht die Knoten des Baumes nach Postorder-Traversierung
void free_tree(height_one_tree *atree) {
	if(atree->root == NULL){
		return;
	}
	if(atree->root->left != NULL){
		free(atree->root->left);
	}
	if(atree->root->right != NULL){
		free(atree->root->right);
	}
	free(atree->root);
   
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
