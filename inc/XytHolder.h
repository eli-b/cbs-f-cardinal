#ifndef XYTHOLDER_H
#define XYTHOLDER_H

#include <algorithm>
#include <forward_list>
#include <tuple>
#include <exception>
#include <iostream>
#include <memory>  // for unique_ptr
#include <vector>
#include <memory_resource>  // For pmr::

/// Stores T instances by location and time.
/// Since every timestep is sparse (most locations at each timestep are unused),
/// for each location, we map the timestep they're used in to the T instance.
template <class T, typename Deleter = std::default_delete<T>>
class XytHolder {
public:
    XytHolder(int xy_size, Deleter deleter_param = std::default_delete<T>())
     : deleter(deleter_param), data(xy_size, nullptr) {}

    Deleter deleter;

//    // FIXME: Must do a deep copy - can't have unique_ptrs pointing to same data...
//    XytHolder(const XytHolder<T>& other) : data(other.data.size(), nullptr) {
//        for (int i = 0; i < other.data.size() ; ++i) {
//            if (other.data[i] != nullptr) {
//                for (const auto& pair: *other.data[i]) {
//                    auto& [time, item] = pair;
//                    set(i, time, item.get());
//                }
//            }
//        }
//    }
	XytHolder(const XytHolder<T,Deleter>& other) = delete;  // For now

	XytHolder(XytHolder<T,Deleter>&& other)
     : count(other.count), deleter(std::move(other.deleter)), data(std::move(other.data))
     {}  // Move constructor

    // TODO: Implement an iterator and begin and end methods:
//    class Iterator {
//        XytHolder* holder;
//        int location;
//        std::list<std::tuple<int,T>>::iterator it;
//    };

    // Returns <true,requested item> or <false,nullptr> when it's missing
    std::tuple<bool, T*> get(int location_id, int t) const {
        // Linear lookup
        if (data[location_id] == nullptr)
            return std::make_tuple(false, nullptr);
        for (auto it = data[location_id]->begin(); it != data[location_id]->end() ; ++it)  {
            auto& [it_t, it_n] = *it;
            if (it_t == t) {
                return std::make_tuple(true, it_n.get());
            }
            else if (it_t > t) {
                return std::make_tuple(false, nullptr);
            }
        }
        return std::make_tuple(false, nullptr);
    }

    std::tuple<bool, int> latest_entry(int location_id) const {
        if (data[location_id] == nullptr)
            return std::make_tuple(false, -1);
        auto& [last_t, last_n] = data[location_id]->back();
        return std::make_tuple(true, last_t);
    }

    void set(int location_id, int t, T* value) {
        ++count;
        // Linear insertion
        if (data[location_id] == nullptr)
			data[location_id] = new std::pmr::forward_list<std::pair<int,std::unique_ptr<T,Deleter>>>();

		auto prev = data[location_id]->begin();
		auto it = data[location_id]->begin();
		for (; it != data[location_id]->end() ; ++it)  {
			auto& [it_t, it_n] = *it;
			if (it_t == t) {
				std::cout << "Unexpected re-insertion of item to XytHolder!" << std::endl;
				std::abort();
			}
			else if (it_t > t) {
				break;  // Insert before <it>
			}
			prev = it;
		}
		if (prev != it)
			data[location_id]->insert_after(prev,
											std::make_pair(t, std::unique_ptr<T,Deleter>(value, deleter)));  // inserts before the iterator
		else  // list is empty or we want to insert before the first item
			data[location_id]->push_front(std::make_pair(t, std::unique_ptr<T,Deleter>(value, deleter)));
    }

    ~XytHolder() {
        clear();
        data.clear();
    }

    void clear() {
        for (size_t i = 0; i < data.size(); ++i) {
            delete data[i];
            data[i] = nullptr;
			// The list's allocator is shared with all lists that use an allocator for the same size,
			// so the allocator will get its memory back and *keep it*. It will not give it back to the system.
        }
        count = 0;
    }

    int count = 0;
//	using time_and_unique_ptr_tuple = std::tuple<int,std::unique_ptr<T, Deleter>>;
	using time_and_unique_ptr_tuple = std::pair<int,std::unique_ptr<T, Deleter>>;
	using list_of_time_and_uniqe_ptr_tuples = std::pmr::forward_list<time_and_unique_ptr_tuple>;
    std::vector<list_of_time_and_uniqe_ptr_tuples*> data;  // nullptrs (8) are three times smaller than empty std::lists (24)
};


#endif //XYTHOLDER_H
