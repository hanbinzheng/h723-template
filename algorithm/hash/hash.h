#ifndef HASH_H_
#define HASH_H_

#include <stdbool.h>
#include <stdint.h>

#define HASH_TABLE_SIZE 64u
#define NUM_BIT_OF_HASH_TABLE_SIZE 6u

enum hash_slot_state {
	HASH_SLOT_EMPTY = 0,
	HASH_SLOT_OCCUPIED,
	HASH_SLOT_DELETED,
};

struct hash_entry {
	enum hash_slot_state state;
	uint32_t key;
	uint32_t val;
};

struct hash_table {
	struct hash_entry entries[HASH_TABLE_SIZE];
	uint16_t count;
};

/**
 * @brief Initialize the hash table, clearing all slots.
 *
 * @param table Pointer to the hash table instance.
 */
void hash_init(struct hash_table *table);

/**
 * @brief Insert or update a key-value pair.
 *
 * @param table Pointer to the hash table instance.
 * @param key 32-bit key (e.g., CAN ID).
 * @param val 32-bit value (e.g., casted function pointer).
 * @return true if successful, false if the table is full.
 */
bool hash_insert(struct hash_table *table, uint32_t key, uint32_t val);

/**
 * @brief Look up the value associated with a given key.
 *
 * @param table Pointer to the hash table instance.
 * @param key The key to look for.
 * @param out Pointer to store the retrieved value.
 * @return true if the key was found, false otherwise.
 */
bool hash_lookup(const struct hash_table *table, uint32_t key, uint32_t *out);

/**
 * @brief Remove a key-value pair from the table.
 *
 * @param table Pointer to the hash table instance.
 * @param key The key to be removed.
 * @return true if successfully removed, false if the key does not exist.
 */
bool hash_remove(struct hash_table *table, uint32_t key);

/**
 * @brief Clear all elements in the hash table.
 *
 * @param table Pointer to the hash table instance.
 */
void hash_clear(struct hash_table *table);

#endif /* HASH_H_ */
