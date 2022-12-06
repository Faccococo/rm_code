# C++

## Language

##### Data Type

- [ ] 基本数据类型（void,bool,各类整型和浮点,指针）
- [ ] 定长整型(int64_t),size_t,ptrdiff_t
- [ ] std::numeric_limits
- [ ] 和类型union
- [ ] 积类型struct
- [ ] array
- [ ] 内存对齐
- [ ] sizeof/alignof/alignas
- [ ] Pointer&Reference
- [ ] nullptr
- [ ] literals of ints/floats/strings
- [ ] user-defined literals
- [ ] const specifier and const_cast

##### Data Flow

- [ ] 整型/浮点的基本运算
- [ ] operator precedence
- [ ] 短路求值&&/||
- [ ] 逗号运算符,和三目运算符?:
- [ ] static_cast and reinterpret_cast

##### Basic Control Flow

- [ ] if/else/else if
- [ ] for
- [ ] switch
- [ ] while/do-while
- [ ] continue/break
- [ ] return

##### Function

- [ ] declaration/definition
- [ ] function pointer/type of function pointer
- [ ] overload

##### Class

- [ ] (static) data members/member functions
- [ ] member access specifiers: private/protected/public/friend
- [ ] this pointer
- [ ] special member functions: default constructor/copy constructor/copy assignment/move constructor/move assignment/destructor
- [ ] default/delete specifier
- [ ] derived class
- [ ] abstract class/virtual function/pure virtual function
- [ ] virtual destructor
- [ ] const member function/mutable specifier
- [ ] rule of 350
- [ ] operator overloading
- [ ] explicit converting constructor

##### Basic Templates

- [ ] template functions
- [ ] template classes
- [ ] template argument deduction
- [ ] explicit specialization
- [ ] auto deduction
- [ ] trait helper class
- [ ] SFINAE(std::enable_if_t/std::is_same_v)
- [ ] dependent names
- [ ] decltype
- [ ] type_traits
- [ ] type alias (typedef and using)

##### Exceptions

- [ ] stack unwinding when exception raises
- [ ] throw/catch
- [ ] std::exception
- [ ] built-in exception types
- [ ] catch(const T&)
- [ ] catch(...)
- [ ] RTTI in catching
- [ ] noexcept specifier
- [ ] rethrow
- [ ] exception in destructor

##### Right value reference/universal reference

- [ ] left value reference vs right value reference
- [ ] pass by value and std::move
- [ ] Universal reference(T&& and auto&&) and std::forward

##### Memory management

- [ ] malloc/free
- [ ] new/delete
- [ ] new[]/delete[]

##### RAII

- [ ] ownership of resources
- [ ] how to write correct copy/move constructor/assignment
- [ ] std::unique_ptr/std::shared_ptr

##### RTTI

- [ ] dynamic_cast
- [ ] typeid and std::type_info

##### Lambda function

- [ ] lambda function definition
- [ ] closure(capture by value/reference/move)
- [ ] capture this
- [ ] std::function and type erasing and recursion

##### Multiple file compilation

- [ ] lifetime of static variables
- [ ] linkage
- [ ] namespaces
- [ ] extern specifier & anonymous namespaces
- [ ] main() entry

##### Miscellaneous

- [ ] structured binding
- [ ] constexpr
- [ ] std::initializer_list
- [ ] assert/static_assert

## Library

##### Utilities & Diagnostics

- [ ] std::abort
- [ ] std::exit
- [ ] std::quick_exit
- [ ] SIGSEGV/SIGFPE/SIGINT

##### Time

- [ ] Clocks/Durations/Time Points
- [ ] std::chrono::high_resolution_clock
- [ ] time_since_epoch

##### String

- [ ] ASCII & Unicode
- [ ] code points & encodings
- [ ] std::string
- [ ] std::to_string/fmt::format
- [ ] std::stringstream

##### Containers & Iterators

- [ ] [begin,end)
- [ ] const iterators and reverse iterators
- [ ] iterator traits
- [ ] range-for
- [ ] sequence containers: list/vector
- [ ] std::unordered_set/std::unordered_map
- [ ] iterator invalidation & Member function table

##### Helper Types

- [ ] std::pair & std::tuple
- [ ] std::any
- [ ] std::optional (std::nullopt)
- [ ] std::variant (std::monostate)

##### Algorithm

- [ ] Non-modifying sequence operations
- [ ] Modifying sequence operations
- [ ] std::sort
- [ ] Minimum/maximum operations

##### IO Library

- [ ] std::istream/std::ostream
- [ ] std::fstream
- [ ] std::cin/std::cout
- [ ] std::endl/std::flush
- [ ] ANSI color escape sequences

##### Filesystem

- [ ] path
- [ ] exists/is_regular_file
- [ ] create/remove files/directories
- [ ] directory_iterator & recursive_directory_iterator

##### Regex

- [ ] regular expressions (grammar & verifying)
- [ ] std::regex
- [ ] std::regex_match
- [ ] std::regex_search

##### Threading

- [ ] std::thread (argument passing, join, detach)
- [ ] std::thread::hardware_concurrency
- [ ] std::this_thread::get_id/sleep/yield
- [ ] std::mutex & std::lock_guard
- [ ] std::shared_mutex & std::shared_lock & std::unique_lock
- [ ] std::atomic_xxx & CAS

##### Numerics

###### cmath

- [ ] abs/fabs
- [ ] fmax/fmin
- [ ] Exponential functions
- [ ] Power functions
- [ ] Trigonometric functions (atan2)
- [ ] Nearest integer floating point operations

###### random

- [ ] engines & distributions & seed
- [ ] std::mt19937_64
- [ ] uniform distribution

## Normalization

- [ ] 代码格式规范
- [ ] 参数传递规范
- [ ] 命名规范

# Visual Studio

- [ ] 构建cmake项目并运行程序
- [ ] 调试/断点/单步/查看中间变量
- [ ] 运行测试

# CMake

- [ ] cmake版本约束
- [ ] cmake全局变量
- [ ] Compiler/OS/BuileType判断
- [ ] 增加executable
- [ ] vcpkg包管理
- [ ] ctest
- [ ] cmake命令行

# vcpkg

- [ ] vcpkg安装/更新/卸载/列举包
- [ ] 集成vcpkg到VS中
- [ ] 集成vcpkg到cmake中

# git

- [ ] gitlab的使用
- [ ] git add/commit/push/pull
- [ ] git merge

# OpenCV

- [ ] RGB/HSV色彩空间

- [ ] image/video读写
- [ ] cv::Mat使用
- [ ] cv::inRange/cv::threshold/cv::cvtColor
- [ ] cv::erode/cv::dilate/cv::findContours
- [ ] PowerToys的ColorPicker使用

# glm

- [ ] OpenGL右手坐标系
- [ ] glm::vec2/glm::vec3/glm::vec4/glm::mat3/glm::mat4的基本使用
- [ ] matrix_transform
- [ ] quaternion glm::quat
- [ ] glm::perspective/glm::lookAt
- [ ] constants.hpp

# CAF

- [ ] actor model
- [ ] actor spawing
- [ ] actor messaging

# documentation

- [ ] 检索cppreference.com
- [ ] 检索opencv文档
- [ ] 检索cmake文档
- [ ] 检索glm文档


