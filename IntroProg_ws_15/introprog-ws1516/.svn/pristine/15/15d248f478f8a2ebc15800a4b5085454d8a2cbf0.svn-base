#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct _node node;
typedef struct _height_one_tree height_one_tree;

struct _height_one_tree {
    node *root;
};

struct _node {
    node *left;
    node *right;
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

// Nur für Bäume mit 3 Knoten, also der Höhe 1
void insert_node(height_one_tree *atree, int value) {
    // Hier Implementieren
    node *element = malloc(sizeof(node));
    element->value = value;
    element->left = NULL;
    element->right = NULL;
	if (atree->root == NULL) {
		atree->root = element;
	} else if (value < atree->root->value) {
		atree->root->left = element;
	} else {
		atree->root->right = element;
	}
}

void free_tree(height_one_tree *atree) {
    // Hier Implementieren

	if (atree->root != NULL) {
		if (atree->root->left != NULL)
			free(atree->root->left);
		if (atree->root->right != NULL)
			free(atree->root->right);
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
