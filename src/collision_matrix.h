#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "macros_utils.h"

namespace mplib {

enum class AllowedCollision {
  NEVER,   // Collision is never allowed
  ALWAYS,  // Collision is always allowed
  // unused for now
  CONDITIONAL,  // Collision contact is allowed depending on a predicate
};

// AllowedCollisionMatrixPtr
MPLIB_CLASS_FORWARD(AllowedCollisionMatrix);

class AllowedCollisionMatrix {
 public:
  AllowedCollisionMatrix();

  /**
   * @brief Get the type of the allowed collision between two elements
   * @returns AllowedCollision, std::nullopt if the entry does not exist.
   */
  std::optional<AllowedCollision> getEntry(const std::string &name1,
                                           const std::string &name2) const;

  /// @brief Check if an entry exists for an element
  bool hasEntry(const std::string &name) const;

  /// @brief Check if an entry exists for a pair of elements
  bool hasEntry(const std::string &name1, const std::string &name2) const;

  /// @brief Set an entry for a pair of elements
  void setEntry(const std::string &name1, const std::string &name2, bool allowed);

  /// @brief Set the entries between the element and each element in other_names
  void setEntry(const std::string &name, const std::vector<std::string> &other_names,
                bool allowed);

  /// @brief Set the entries for all possible pairs among two sets of elements
  void setEntry(const std::vector<std::string> &names1,
                const std::vector<std::string> &names2, bool allowed);

  /**
   * @brief Set the entries for all possible pairs between the element and
   *  existing elements. As the set of elements might change in the future,
   *  consider using setDefaultEntry() instead.
   */
  void setEntry(const std::string &name, bool allowed);

  /**
   * @brief Set the entries for all possible pairs between each of the elements
   *  and existing elements. As the set of elements might change in the
   *  future, consider using setDefaultEntry() instead.
   */
  void setEntry(const std::vector<std::string> &names, bool allowed);

  /// @brief Set the entries for all possible pairs among all existing elements
  void setEntry(bool allowed);

  /// @brief Remove the entry for a pair of elements if exists
  void removeEntry(const std::string &name1, const std::string &name2);

  /**
   * @brief Remove existing entries between the element and each element in
   *  other_names
   */
  void removeEntry(const std::string &name,
                   const std::vector<std::string> &other_names);

  /**
   * @brief Remove any existing entries for all possible pairs among two sets of
   *  elements
   */
  void removeEntry(const std::vector<std::string> &names1,
                   const std::vector<std::string> &names2);

  /**
   * @brief Remove all entries for all possible pairs between the element and
   *  existing elements
   */
  void removeEntry(const std::string &name);

  /**
   * @brief Remove all entries for all possible pairs between each of the
   *  elements and existing elements
   */
  void removeEntry(const std::vector<std::string> &names);

  /// @brief Get the size of the allowed collision matrix (number of entries)
  size_t getSize() const { return entries_.size(); }

  /**
   * @brief Get the default type of the allowed collision for an element
   * @returns AllowedCollision, std::nullopt if the default entry does not exist
   */
  std::optional<AllowedCollision> getDefaultEntry(const std::string &name) const;

  /// @brief Check if a default entry exists for an element
  bool hasDefaultEntry(const std::string &name) const;

  /**
   * @brief Set the default value for entries that include name but are not set
   *  explicitly with setEntry(). Apply to future changes of the element set.
   */
  void setDefaultEntry(const std::string &name, bool allowed);

  /**
   * @brief Set the default entries for the elements. Apply to future changes of
   *  the element set.
   */
  void setDefaultEntry(const std::vector<std::string> &names, bool allowed);

  /// @brief Remove the default entry for the element if exists
  void removeDefaultEntry(const std::string &name);

  /// @brief Remove the existing default entries for the elements
  void removeDefaultEntry(const std::vector<std::string> &names);

  /**
   * @brief Get the type of the allowed collision between two elements
   * @returns AllowedCollision. This is
   *  * std::nullopt if the entry does not exist (collision is not allowed)
   *  * the entry if an entry or a default entry exists.
   */
  std::optional<AllowedCollision> getAllowedCollision(const std::string &name1,
                                                      const std::string &name2) const;

  /// @brief Clear all data in the allowed collision matrix
  void clear();

  /**
   * @brief Get sorted names of all existing elements (including
   *  default_entries_)
   */
  std::vector<std::string> getAllEntryNames() const;

  /**
   * @brief Print the allowed collision matrix
   *  "01?-" corresponds to NEVER / ALWAYS / CONDITIONAL / Entry not found
   */
  void print(std::ostream &out) const;

 private:
  /**
   * @brief Get the default type of the allowed collision for a pair of elements
   * @returns AllowedCollision
   *  If neither element has a default entry, returns std::nullopt.
   *  If only one of the elements has a default entry, returns the entry.
   *  If both elements have default entries, returns
   *    * NEVER if at least one of the default types is NEVER;
   *    * CONDITIONAL if at least one of the default types is CONDITIONAL;
   *    * ALWAYS otherwise.
   */
  std::optional<AllowedCollision> getDefaultEntry(const std::string &name1,
                                                  const std::string &name2) const;

  std::unordered_map<std::string, std::unordered_map<std::string, AllowedCollision>>
      entries_;
  std::unordered_map<std::string, AllowedCollision> default_entries_;
};

}  // namespace mplib
