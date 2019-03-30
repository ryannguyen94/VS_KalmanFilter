template <class T> class Vec {

public:
	typedef T* iterator;
	typedef const T* const_iterator;
	typedef size_t size_type;
	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;

	Vec() { create(); }
	explicit Vec(size_type n, const T& t = T()) { create(n, t); }
	Vec(const Vec& v) { create(v.begin(), v.end()); }	// copy constructor
	Vec& operator=(const Vec&);
	T& operator[](size_type i) { return data[i]; }
	const T& operator[](size_type i) const { return data[i]; }

	void push_back(const T& t)
	{
		if (avail == limit)
			grow();
		unchecked_append(t);
	}

	size_type size() const { return avail - data; }
	iterator begin() { return data; }
	const_iterator begin() const { return data; }

	iterator end() { return avail; }
	const_iterator end() const { return avail; }

private:

};