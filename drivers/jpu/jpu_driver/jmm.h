/* SPDX-License-Identifier: BSD-3-Clause */
//-----------------------------------------------------------------------------
// COPYRIGHT (C) 2020   CHIPS&MEDIA INC. ALL RIGHTS RESERVED
//
// This file is distributed under BSD 3 clause and LGPL2.1 (dual license)
// SPDX License Identifier: BSD-3-Clause
// SPDX License Identifier: LGPL-2.1-only
//
// The entire notice above must be reproduced on all authorized copies.
//
// Description  :
//-----------------------------------------------------------------------------
#ifndef __CNM_JPU_MM_H__
#define __CNM_JPU_MM_H__

struct jmem_key_t {
	unsigned long long key;
};

#define JMEM_PAGE_SIZE (16 * 1024)
#define KEY_TO_VALUE(_key) (_key.key >> 32)
static inline struct jmem_key_t MAKE_KEY(unsigned long long _a,
					 unsigned long long _b) {
	struct jmem_key_t key = { (((unsigned long long)_a) << 32 | _b) };
	return key;
}

struct page_t {
	int pageno;
	unsigned long addr;
	int used;
	int alloc_pages;
	int first_pageno;
};

struct avl_node_t {
	struct jmem_key_t key;
	int height;
	struct page_t *page;
	struct avl_node_t *left;
	struct avl_node_t *right;
};

struct jpu_mm_t {
	struct avl_node_t *free_tree;
	struct avl_node_t *alloc_tree;
	struct page_t *page_list;
	int num_pages;
	unsigned long base_addr;
	unsigned long mem_size;
	int free_page_count;
	int alloc_page_count;
};

#define VMEM_P_ALLOC(_x) kmalloc(_x, GFP_KERNEL)
#define VMEM_P_FREE(_x) kfree(_x)

#define VMEM_ASSERT(_exp)                                                      \
	do {                                                                   \
		if (!(_exp)) {                                                 \
			dev_info(jpu_devp, "VMEM_ASSERT at %s:%d\n", __FILE__, \
				 __LINE__); /*while(1);*/                      \
		}                                                              \
	} while (0)
#define VMEM_HEIGHT(_tree) (_tree == NULL ? -1 : _tree->height)

#define MAX(_a, _b) (_a >= _b ? _a : _b)

enum rotation_dir_t { LEFT, RIGHT };

struct avl_node_data_t {
	int key;
	struct page_t *page;
};

static struct avl_node_t *make_avl_node(struct jmem_key_t key,
					struct page_t *page)
{
	struct avl_node_t *node =
		(struct avl_node_t *)VMEM_P_ALLOC(sizeof(struct avl_node_t));
	node->key = key;
	node->page = page;
	node->height = 0;
	node->left = NULL;
	node->right = NULL;

	return node;
}

static int get_balance_factor(struct avl_node_t *tree)
{
	int factor = 0;

	if (tree)
		factor = VMEM_HEIGHT(tree->right) - VMEM_HEIGHT(tree->left);

	return factor;
}

/*
 * Left Rotation
 *
 *      A                      B
 *       \                    / \
 *        B         =>       A   C
 *       /  \                 \
 *      D    C                 D
 *
 */
static struct avl_node_t *rotation_left(struct avl_node_t *tree)
{
	struct avl_node_t *rchild;
	struct avl_node_t *lchild;

	if (tree == NULL)
		return NULL;

	rchild = tree->right;
	if (rchild == NULL)
		return tree;

	lchild = rchild->left;
	rchild->left = tree;
	tree->right = lchild;

	tree->height =
		MAX(VMEM_HEIGHT(tree->left), VMEM_HEIGHT(tree->right)) + 1;
	rchild->height =
		MAX(VMEM_HEIGHT(rchild->left), VMEM_HEIGHT(rchild->right)) + 1;

	return rchild;
}

/*
 * Reft Rotation
 *
 *         A                  B
 *       \                  /  \
 *      B         =>       D    A
 *    /  \                     /
 *   D    C                   C
 *
 */
static struct avl_node_t *rotation_right(struct avl_node_t *tree)
{
	struct avl_node_t *rchild;
	struct avl_node_t *lchild;

	if (tree == NULL)
		return NULL;

	lchild = tree->left;
	if (lchild == NULL)
		return NULL;

	rchild = lchild->right;
	lchild->right = tree;
	tree->left = rchild;

	tree->height =
		MAX(VMEM_HEIGHT(tree->left), VMEM_HEIGHT(tree->right)) + 1;
	lchild->height =
		MAX(VMEM_HEIGHT(lchild->left), VMEM_HEIGHT(lchild->right)) + 1;

	return lchild;
}

static struct avl_node_t *do_balance(struct avl_node_t *tree)
{
	int bfactor = 0, child_bfactor; /* balancing factor */

	bfactor = get_balance_factor(tree);

	if (bfactor >= 2) {
		child_bfactor = get_balance_factor(tree->right);
		if (child_bfactor == 1 || child_bfactor == 0) {
			tree = rotation_left(tree);
		} else if (child_bfactor == -1) {
			tree->right = rotation_right(tree->right);
			tree = rotation_left(tree);
		} else {
			dev_info(jpu_devp, "invalid balancing factor: %d\n",
				 child_bfactor);
			VMEM_ASSERT(0);
			return NULL;
		}
	} else if (bfactor <= -2) {
		child_bfactor = get_balance_factor(tree->left);
		if (child_bfactor == -1 || child_bfactor == 0) {
			tree = rotation_right(tree);
		} else if (child_bfactor == 1) {
			tree->left = rotation_left(tree->left);
			tree = rotation_right(tree);
		} else {
			dev_info(jpu_devp, "invalid balancing factor: %d\n",
				 child_bfactor);
			VMEM_ASSERT(0);
			return NULL;
		}
	}

	return tree;
}
static struct avl_node_t *unlink_end_node(struct avl_node_t *tree, int dir,
					  struct avl_node_t **found_node)
{
	struct avl_node_t *node;
	*found_node = NULL;

	if (tree == NULL)
		return NULL;

	if (dir == LEFT) {
		if (tree->left == NULL) {
			*found_node = tree;
			return NULL;
		}
	} else {
		if (tree->right == NULL) {
			*found_node = tree;
			return NULL;
		}
	}

	if (dir == LEFT) {
		node = tree->left;
		tree->left = unlink_end_node(tree->left, LEFT, found_node);
		if (tree->left == NULL) {
			tree->left = (*found_node)->right;
			(*found_node)->left = NULL;
			(*found_node)->right = NULL;
		}
	} else {
		node = tree->right;
		tree->right = unlink_end_node(tree->right, RIGHT, found_node);
		if (tree->right == NULL) {
			tree->right = (*found_node)->left;
			(*found_node)->left = NULL;
			(*found_node)->right = NULL;
		}
	}

	tree->height =
		MAX(VMEM_HEIGHT(tree->left), VMEM_HEIGHT(tree->right)) + 1;

	return do_balance(tree);
}

static struct avl_node_t *avltree_insert(struct avl_node_t *tree,
					 struct jmem_key_t key,
					 struct page_t *page)
{
	if (tree == NULL) {
		tree = make_avl_node(key, page);
	} else {
		if (key >= tree->key)
			tree->right = avltree_insert(tree->right, key, page);
		else
			tree->left = avltree_insert(tree->left, key, page);
	}

	tree = do_balance(tree);

	tree->height =
		MAX(VMEM_HEIGHT(tree->left), VMEM_HEIGHT(tree->right)) + 1;

	return tree;
}

static struct avl_node_t *do_unlink(struct avl_node_t *tree)
{
	struct avl_node_t *node;
	struct avl_node_t *end_node;

	node = unlink_end_node(tree->right, LEFT, &end_node);
	if (node) {
		tree->right = node;
	} else {
		node = unlink_end_node(tree->left, RIGHT, &end_node);
		if (node)
			tree->left = node;
	}

	if (node == NULL) {
		node = tree->right ? tree->right : tree->left;
		end_node = node;
	}

	if (end_node) {
		end_node->left = (tree->left != end_node) ? tree->left :
							    end_node->left;
		end_node->right = (tree->right != end_node) ? tree->right :
							      end_node->right;
		end_node->height = MAX(VMEM_HEIGHT(end_node->left),
				       VMEM_HEIGHT(end_node->right)) +
				   1;
	}

	tree = end_node;

	return tree;
}

static struct avl_node_t *avltree_remove(struct avl_node_t *tree,
					 struct avl_node_t **found_node,
					 struct jmem_key_t key)
{
	*found_node = NULL;
	if (tree == NULL) {
		dev_info(jpu_devp, "failed to find key %d\n", (int)key);
		return NULL;
	}

	if (key == tree->key) {
		*found_node = tree;
		tree = do_unlink(tree);
	} else if (key > tree->key) {
		tree->right = avltree_remove(tree->right, found_node, key);
	} else {
		tree->left = avltree_remove(tree->left, found_node, key);
	}

	if (tree)
		tree->height =
			MAX(VMEM_HEIGHT(tree->left), VMEM_HEIGHT(tree->right)) +
			1;

	tree = do_balance(tree);

	return tree;
}

void avltree_free(struct avl_node_t *tree)
{
	if (tree == NULL)
		return;
	if (tree->left == NULL && tree->right == NULL) {
		VMEM_P_FREE(tree);
		return;
	}

	avltree_free(tree->left);
	tree->left = NULL;
	avltree_free(tree->right);
	tree->right = NULL;
}

static struct avl_node_t *remove_approx_value(struct avl_node_t *tree,
					      struct avl_node_t **found,
					      struct jmem_key_t key)
{
	*found = NULL;
	if (tree == NULL)
		return NULL;

	if (key == tree->key) {
		*found = tree;
		tree = do_unlink(tree);
	} else if (key > tree->key) {
		tree->right = remove_approx_value(tree->right, found, key);
	} else {
		tree->left = remove_approx_value(tree->left, found, key);
		if (*found == NULL) {
			*found = tree;
			tree = do_unlink(tree);
		}
	}
	if (tree)
		tree->height =
			MAX(VMEM_HEIGHT(tree->left), VMEM_HEIGHT(tree->right)) +
			1;
	tree = do_balance(tree);

	return tree;
}

static void set_blocks_free(struct jpu_mm_t *mm, int pageno, int npages)
{
	int last_pageno = pageno + npages - 1;
	int i;
	struct page_t *page;
	struct page_t *last_page;

	VMEM_ASSERT(npages);

	if (last_pageno >= mm->num_pages) {
		dev_info(jpu_devp, "%s: invalid last page number: %d\n",
			 __func__, last_pageno);
		VMEM_ASSERT(0);
		return;
	}

	for (i = pageno; i <= last_pageno; i++) {
		mm->page_list[i].used = 0;
		mm->page_list[i].alloc_pages = 0;
		mm->page_list[i].first_pageno = -1;
	}

	page = &mm->page_list[pageno];
	page->alloc_pages = npages;
	last_page = &mm->page_list[last_pageno];
	last_page->first_pageno = pageno;

	mm->free_tree =
		avltree_insert(mm->free_tree, MAKE_KEY(npages, pageno), page);
}

static void set_blocks_alloc(struct jpu_mm_t *mm, int pageno, int npages)
{
	int last_pageno = pageno + npages - 1;
	int i;
	struct page_t *page;
	struct page_t *last_page;

	if (last_pageno >= mm->num_pages) {
		dev_info(jpu_devp,
			 "set_blocks_free: invalid last page number: %d\n",
			 last_pageno);
		VMEM_ASSERT(0);
		return;
	}

	for (i = pageno; i <= last_pageno; i++) {
		mm->page_list[i].used = 1;
		mm->page_list[i].alloc_pages = 0;
		mm->page_list[i].first_pageno = -1;
	}

	page = &mm->page_list[pageno];
	page->alloc_pages = npages;

	last_page = &mm->page_list[last_pageno];
	last_page->first_pageno = pageno;

	mm->alloc_tree =
		avltree_insert(mm->alloc_tree, MAKE_KEY(page->addr, 0), page);
}

int jmem_init(struct jpu_mm_t *mm, unsigned long addr, unsigned long size)
{
	int i;

	mm->base_addr = (addr + (JMEM_PAGE_SIZE - 1)) & ~(JMEM_PAGE_SIZE - 1);
	mm->mem_size = size & ~JMEM_PAGE_SIZE;
	mm->num_pages = mm->mem_size / JMEM_PAGE_SIZE;
	mm->page_list = (struct page_t *)VMEM_P_ALLOC(mm->num_pages *
						      sizeof(struct page_t));
	mm->free_tree = NULL;
	mm->alloc_tree = NULL;
	mm->free_page_count = mm->num_pages;
	mm->alloc_page_count = 0;

	for (i = 0; i < mm->num_pages; i++) {
		mm->page_list[i].pageno = i;
		mm->page_list[i].addr = mm->base_addr + i * JMEM_PAGE_SIZE;
		mm->page_list[i].alloc_pages = 0;
		mm->page_list[i].used = 0;
		mm->page_list[i].first_pageno = -1;
	}

	set_blocks_free(mm, 0, mm->num_pages);

	return 0;
}

int jmem_exit(struct jpu_mm_t *mm)
{
	if (mm == NULL) {
		dev_info(jpu_devp, "vmem_exit: invalid handle\n");
		return -1;
	}

	if (mm->free_tree)
		avltree_free(mm->free_tree);

	if (mm->alloc_tree)
		avltree_free(mm->alloc_tree);

	VMEM_P_FREE(mm->page_list);

	mm->base_addr = 0;
	mm->mem_size = 0;
	mm->num_pages = 0;
	mm->page_list = NULL;
	mm->free_tree = NULL;
	mm->alloc_tree = NULL;
	mm->free_page_count = 0;
	mm->alloc_page_count = 0;
	return 0;
}

unsigned long jmem_alloc(struct jpu_mm_t *mm, int size, unsigned long pid)
{
	struct avl_node_t *node;
	struct page_t *free_page;
	int npages, free_size;
	int alloc_pageno;
	unsigned long ptr;

	if (mm == NULL) {
		dev_info(jpu_devp, "vmem_alloc: invalid handle\n");
		return -1;
	}

	if (size <= 0)
		return -1;

	npages = (size + JMEM_PAGE_SIZE - 1) / JMEM_PAGE_SIZE;

	mm->free_tree =
		remove_approx_value(mm->free_tree, &node, MAKE_KEY(npages, 0));
	if (node == NULL)
		return -1;
	free_page = node->page;
	free_size = KEY_TO_VALUE(node->key);

	alloc_pageno = free_page->pageno;
	set_blocks_alloc(mm, alloc_pageno, npages);
	if (npages != free_size) {
		int free_pageno = alloc_pageno + npages;

		set_blocks_free(mm, free_pageno, (free_size - npages));
	}

	VMEM_P_FREE(node);

	ptr = mm->page_list[alloc_pageno].addr;
	mm->alloc_page_count += npages;
	mm->free_page_count -= npages;

	return ptr;
}

int jmem_free(struct jpu_mm_t *mm, unsigned long ptr, unsigned long pid)
{
	unsigned long addr;
	struct avl_node_t *found;
	struct page_t *page;
	int pageno, prev_free_pageno, next_free_pageno;
	int prev_size, next_size;
	int merge_page_no, merge_page_size, free_page_size;

	if (mm == NULL) {
		dev_info(jpu_devp, "vmem_free: invalid handle\n");
		return -1;
	}

	addr = ptr;

	mm->alloc_tree =
		avltree_remove(mm->alloc_tree, &found, MAKE_KEY(addr, 0));
	if (found == NULL) {
		dev_info(jpu_devp, "vmem_free: 0x%08x not found\n", (int)addr);
		VMEM_ASSERT(0);
		return -1;
	}

	/* find previous free block */
	page = found->page;
	pageno = page->pageno;
	free_page_size = page->alloc_pages;
	prev_free_pageno = pageno - 1;
	prev_size = -1;
	if (prev_free_pageno >= 0) {
		if (mm->page_list[prev_free_pageno].used == 0) {
			prev_free_pageno =
				mm->page_list[prev_free_pageno].first_pageno;
			prev_size = mm->page_list[prev_free_pageno].alloc_pages;
		}
	}

	/* find next free block */
	next_free_pageno = pageno + page->alloc_pages;
	next_free_pageno =
		(next_free_pageno == mm->num_pages) ? -1 : next_free_pageno;
	next_size = -1;
	if (next_free_pageno >= 0)
		if (mm->page_list[next_free_pageno].used == 0)
			next_size = mm->page_list[next_free_pageno].alloc_pages;
	VMEM_P_FREE(found);

	/* merge */
	merge_page_no = page->pageno;
	merge_page_size = page->alloc_pages;
	if (prev_size >= 0) {
		mm->free_tree =
			avltree_remove(mm->free_tree, &found,
				       MAKE_KEY(prev_size, prev_free_pageno));
		if (found == NULL) {
			VMEM_ASSERT(0);
			return -1;
		}
		merge_page_no = found->page->pageno;
		merge_page_size += found->page->alloc_pages;
		VMEM_P_FREE(found);
	}
	if (next_size >= 0) {
		mm->free_tree =
			avltree_remove(mm->free_tree, &found,
				       MAKE_KEY(next_size, next_free_pageno));
		if (found == NULL) {
			VMEM_ASSERT(0);
			return -1;
		}
		merge_page_size += found->page->alloc_pages;
		VMEM_P_FREE(found);
	}

	page->alloc_pages = 0;
	page->first_pageno = -1;

	set_blocks_free(mm, merge_page_no, merge_page_size);

	mm->alloc_page_count -= free_page_size;
	mm->free_page_count += free_page_size;

	return 0;
}

#endif /* __CNM_JPU_MM_H__ */
