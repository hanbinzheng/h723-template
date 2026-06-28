/**
 * @file hash.c
 * @brief Open-addressing linear probing hash table implementation.
 */

#include "hash.h"
#include <string.h>

void hash_init(struct hash_table *table)
{
	if (table != NULL)
		memset(table, 0, sizeof(struct hash_table));
}

void hash_clear(struct hash_table *table)
{
	hash_init(table);
}

/* simple and quick hash function */
__ALWAYS_INLINE
static uint32_t hash(uint32_t key)
{
	uint32_t h = key * 0x9E3779B9; /* magic number */
	return (h >> (32u - NUM_BIT_OF_HASH_TABLE_SIZE));
}

bool hash_insert(struct hash_table *table, uint32_t key, uint32_t val)
{
	if (table == NULL) {
		return false;
	}

	uint32_t base_idx = hash(key);
	struct hash_entry *entry = NULL;
	struct hash_entry *insert_slot = NULL;

	for (uint16_t i = 0; i < HASH_TABLE_SIZE; i++) {
		entry = table->entries + ((base_idx + i) & (HASH_TABLE_SIZE - 1));

		/* empty guarantees the key doesn't exist further down */
		if (entry->state == HASH_SLOT_EMPTY && insert_slot == NULL) {
			/* empty, insert directly */
			insert_slot = entry;
			break;
		}

		/* first meet deleted: must continue to check if key exists later */
		if (entry->state == HASH_SLOT_DELETED && insert_slot == NULL) {
			insert_slot = entry; /* first possible position */
			continue;
		}

		/* key exists, update the value and return */
		if (entry->state == HASH_SLOT_OCCUPIED && entry->key == key) {
			entry->val = val;
			return true;
		}
	}

	/* insert the data */
	if (insert_slot != NULL) {
		/* table is full */
		if (table->count >= HASH_TABLE_SIZE) {
			return false;
		}

		insert_slot->state = HASH_SLOT_OCCUPIED;
		insert_slot->key = key;
		insert_slot->val = val;
		table->count++;
		return true;
	}

	return false;
}

__ITCM_FUNC
bool hash_lookup(const struct hash_table *table, uint32_t key, uint32_t *out)
{
	if (table == NULL || out == NULL) {
		return false;
	}

	uint32_t base_idx = hash(key);
	const struct hash_entry *entry = NULL;

	for (uint16_t i = 0; i < HASH_TABLE_SIZE; i++) {
		/* linear probing, use bitwise and to simulate mod operation */
		entry = table->entries + ((base_idx + i) & (HASH_TABLE_SIZE - 1));

		if (entry->state == HASH_SLOT_EMPTY) {
			return false;
		}

		if (entry->state == HASH_SLOT_OCCUPIED && entry->key == key) {
			*out = entry->val;
			return true;
		}
	}

	return false;
}

bool hash_remove(struct hash_table *table, uint32_t key)
{
	if (table == NULL) {
		return false;
	}

	uint32_t base_idx = hash(key);
	struct hash_entry *entry = NULL;

	for (uint16_t i = 0; i < HASH_TABLE_SIZE; i++) {
		entry = table->entries + ((base_idx + i) & (HASH_TABLE_SIZE - 1));

		if (entry->state == HASH_SLOT_EMPTY) {
			return false;
		}

		if (entry->state == HASH_SLOT_OCCUPIED && entry->key == key) {
			entry->state = HASH_SLOT_DELETED;
			table->count--;
			return true;
		}
	}
	return false;
}
