/// @file       src/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    This file contains the definition of the Node class.
///             The Node class represents a node in the network. It is templated by the type
///             of the node's id, which must be an unsigned integral type.

#ifndef Node_hpp
#define Node_hpp

#include <functional>
#include <queue>
#include <utility>
#include <string>
#include <stdexcept>

namespace dsm {
  /// @brief The Node class represents a node in the network.
  /// @tparam Id The type of the node's id. It must be an unsigned integral type.
  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  class Node {
  private:
    std::queue<Id> m_queue;
    std::pair<double, double> m_coords;
    Id m_id;
    Size m_capacity;

  public:
    Node() = default;
    /// @brief Construct a new Node object
    /// @param id The node's id
    explicit Node(Id id);
    /// @brief Construct a new Node object
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates
    Node(Id id, std::pair<double, double> coords);
    /// @brief Construct a new Node object
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates
    /// @param queue A std::queue containing the node's queue
    Node(Id id, std::pair<double, double> coords, std::queue<Id> queue);

    /// @brief Set the node's coordinates
    /// @param coords A std::pair containing the node's coordinates
    void setCoords(std::pair<double, double> coords);
    /// @brief Set the node's queue
    /// @param queue A std::queue containing the node's queue
    /// @throw std::invalid_argument if the queue size is greater than the node's capacity
    void setQueue(std::queue<Id> queue);
    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    void setCapacity(Size capacity);
    /// @brief Enqueue an id to the node's queue
    /// @param id The id to enqueue
    /// @throw std::runtime_error if the queue is full
    void enqueue(Id id);
    /// @brief Dequeue an id from the node's queue
    /// @return Id The dequeued id
    /// @throw std::runtime_error if the queue is empty
    Id dequeue();

    /// @brief Get the node's id
    /// @return Id, The node's id
    Id id() const;
    /// @brief Get the node's coordinates
    /// @return std::pair<double,, double> A std::pair containing the node's coordinates
    const std::pair<double, double>& coords() const;
    /// @brief Get the node's queue
    /// @return std::queue<Id>, A std::queue containing the node's queue
    const std::queue<Id>& queue() const;
    /// @brief Get the node's queue capacity
    /// @return Size The node's queue capacity
    Size capacity() const;
    /// @brief Returns true if the node's queue is full
    /// @return bool True if the node's queue is full
    bool isFull() const;
  };

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  Node<Id, Size>::Node(Id id) : m_id{id}, m_capacity{1} {}

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  Node<Id, Size>::Node(Id id, std::pair<double, double> coords)
      : m_coords{std::move(coords)}, m_id{id}, m_capacity{1} {}

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  Node<Id, Size>::Node(Id id, std::pair<double, double> coords, std::queue<Id> queue)
      : m_queue{std::move(queue)}, m_coords{std::move(coords)}, m_id{id}, m_capacity{1} {}

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  Id Node<Id, Size>::id() const {
    return m_id;
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  void Node<Id, Size>::setCoords(std::pair<double, double> coords) {
    m_coords = std::move(coords);
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  void Node<Id, Size>::setQueue(std::queue<Id> queue) {
    if (queue.size() > m_capacity) {
      std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                           "Node's queue capacity is smaller than the queue size"};
      throw std::invalid_argument(errorMsg);
    }
    m_queue = std::move(queue);
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  void Node<Id, Size>::setCapacity(Size capacity) {
    if (capacity < m_queue.size()) {
      std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                           "Node's queue capacity is smaller than the current queue size"};
      throw std::invalid_argument(errorMsg);
    }
    m_capacity = capacity;
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  void Node<Id, Size>::enqueue(Id id) {
    if (m_queue.size() == m_capacity) {
      std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                           "Node's queue is fulls"};
      throw std::runtime_error(errorMsg);
    }
    m_queue.push(id);
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  Id Node<Id, Size>::dequeue() {
    if (m_queue.empty()) {
      std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                           "Node's queue is empty"};
      throw std::runtime_error(errorMsg);
    }
    Id id = m_queue.front();
    m_queue.pop();
    return id;
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  const std::pair<double, double>& Node<Id, Size>::coords() const {
    return m_coords;
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  const std::queue<Id>& Node<Id, Size>::queue() const {
    return m_queue;
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  Size Node<Id, Size>::capacity() const {
    return m_capacity;
  }

  template <typename Id, typename Size>
    requires std::unsigned_integral<Id> && std::unsigned_integral<Size>
  bool Node<Id, Size>::isFull() const {
    return m_queue.size() == m_capacity;
  }

  // to be implemented
  /* template <typename Id> */
  /* class Intersection : public Node<Id, Size> { */
  /* private: */
  /*   std::function<void()> m_priority; */
  /* }; */

  /* template <typename Id> */
  /* class Roundabout : public Node<Id, Size> { */
  /* private: */
  /*   std::function<void()> m_priority; */
  /* }; */

  /* template <typename Id> */
  /* class TrafficLight : public Node<Id, Size> { */
  /* private: */
  /*   std::function<void()> m_priority; */
  /* }; */
};  // namespace dsm

#endif
