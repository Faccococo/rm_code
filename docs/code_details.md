# Code Details

##  HubClassRegister

+ 宏定义中`#`的作用

    + 一个`#`:将其后面的宏参数进行字符串化

    + 两个`##`:在带参数数的宏定义中将两个子串(token)连接起来，从而形成一个个新的字串。子串(token)指编译器能够识别的最小语法单元。

        ```c++
        #define PRINT(N) printf("token"#N" = %d", token##N)
        
        int token9 = 3;
        PRINT(N)   //printf("token9 = %d", token9)
        
        ```

        总之，宏中的参数仅仅只有一个替换的作用。

+ 初始化时机
    
    + **全局变量、文件域中的静态变量、类中的成员静态变量在main函数执行前初始化；局部变量中的静态变量在第一次调用时初始化。**

```c++
//所有actor类由于声明定义了一个static静态变量的HubClassRegister类， 因此都会在进入main函数之前初始化一个hubClassRegister##CLASS_NAME的变量。
#define HUB_REGISTER_CLASS(CLASS_NAME) static detail::HubClassRegister<CLASS_NAME> hubClassRegister##CLASS_NAME

//HubClassRegister类的构造函数中，调用registerComponent函数，将acotor类的名称、生成的方法注册到工厂类的mClasser变量中，由工厂类负责生成actor类对象。
void registerComponent(const char* name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
    NodeFactory::get().addNodeType(std::string{ name }, std::move(spawnFunction));
}

template <typename NodeType>
class HubClassRegister final : Unmovable {
    public:
    HubClassRegister() {
        registerComponent(typeid(NodeType).name(), [](caf::actor_system& system, const HubConfig& config) -> caf::actor {
            return system.spawn<NodeType>(config);
        });
    }
};

```

## NodeFactory

```c++
//工厂类，制造生成acotr
class NodeFactory final : Unmovable {
    //存放actor的名称和制造acotr的方法
    std::unordered_map<std::string, std::function<caf::actor(caf::actor_system&, const HubConfig&)>> mClasses{};

public:
    //由HubClassRegister类调用，把actor的名称和制造方法存放到mClass中。
    void addNodeType(std::string name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction);
 	//根据配置文件中给定actor的type，在mClasser中寻找与其名称相对的actor，并调用其制造方法生成该actor，并将其注册到系统中。
    caf::actor buildNode(caf::actor_system& system, const std::string& name, const HubConfig& config);
   //单例模式
    static NodeFactory& get() {
        static NodeFactory instance;
        return instance;
    }
};
```

## Atom

```C++
/*本质上为一个结构体
例如：CAF_ADD_ATOM(ArtinxHub, image_frame_atom)
该宏展开为：
struct image_frame_atom{};
static constexpr image_frame_atom image_frame_atom_v = image_frame_atom{};
...
*/
#  define CAF_ADD_ATOM(...)                                                    \
    CAF_PP_OVERLOAD(CAF_ADD_ATOM_, __VA_ARGS__)(__VA_ARGS__)
#endif 
```

## Identifier

```c++
//代码中的key就是一个Identifier结构体，本质是一个64位整型值
//key的生成见HubHelper类
struct Identifier {
    uint64_t val;
};

//带上BlackBoard上的数据类型进行ACTOR_PROTOCOL_CHECK
template <typename T>
struct TypedIdentifier final : Identifier {
    using Payload = T;
};
```

## parseSucceed

```C++
   /**
   * @brief				根据config文件解析actor类中的atom对应的需要发送到的actor
   * @param config 		配置文件
   * @param name		atom的名字
   * @retrun			需要发送到的actor的名称
   **/
std::vector<std::string> parseSucceed(const HubConfig& config, const std::string& name) {
    std::string_view nameNormalized = name;
    //由于name由typeid.name(atom) 传入， name会带上struct关键，所以调用demangle函数去掉
    demangle(nameNormalized);
    const auto attr = config.to_dictionary().value();
    //根据atom名称在config文件中查找
    const auto iter = attr.find(nameNormalized);
    if(iter == attr.cend()) {
        return {};
    }

    const auto succeed = iter->second.to_list().value();
    std::vector<std::string> res;
    res.reserve(succeed.size());
    //将config文件中的发送到目标actor的名称以字符串形式保存
    for(const auto& id : succeed) {
        res.push_back(caf::to_string(id));
    }
    return res;
}

   /**
   * @brief				根据actor名称返回actor的地址和组码
   * @param string		actor的名称
   * @retrun			acotr的地址和组码
   **/
    std::vector<std::pair<caf::actor_addr, GroupMask>> parseSucceed(caf::actor_system& system, const std::vector<std::string>& succeed) {
        const auto& registry = system.registry();
        std::vector<std::pair<caf::actor_addr, GroupMask>> res;
        res.reserve(succeed.size());
        for(const auto& id : succeed) {
            if(const auto addr = registry.get<caf::actor_addr>(id))
                res.emplace_back(addr, maskLUT[id]);
            else {
                logError("Undefined actor " + id + " (call sendAll before start_atom?)");
            }
        }
        return res;
    }
```

## HubHelper

+ `std::variant`
    + 类似C语言中的union，保存可能存储的类型列表之一的对象。
+ `noexcept`
    + 修饰函数表示不会抛出异常。
+ `reinterpre_cast`
    + 强制转型，用来处理无关类型之间的转换， 它会产生一个新的值，这个值会有与原始参数值有完全相同的比特位。
+ `std::get(std::tuple)`
    + `std::get<N>(t)` 提取`tuple`的第N个元素
    + `std::get<typename>(t)`提取`tuple`中`typename`类型的元素，如果`tuple`中不只一个该类型元素，则编译失败。
+ `std::decay_t`
    + 返回去除`cv`属性的类型

```c++
//T: 继承的actor基类    config: 配置文件参数   Succeed: 需要发送的atom
template <typename T, typename Config, typename... Succeed>
class HubHelper : public T {
    //静态断言，编译期判断T的基类是否为caf::abstract_actor
    static_assert(std::is_base_of_v<caf::abstract_actor, T>);

    //模板参数Lable指明了需要发送Atom的类型
    template <typename Label>
    struct SucceedAddress final {
        std::variant<std::vector<std::string>, std::vector<std::pair<caf::actor_addr, GroupMask>>> val;
    };
	
    template <typename Arg>
    static const Arg& wrap(const Arg& arg) noexcept {
        return arg;
    }
	
    //抹掉TypedIdentifier的类型
    template <typename Arg>
    static const Identifier& wrap(const TypedIdentifier<Arg>& arg) noexcept {
        return static_cast<const Identifier&>(arg);
    }

    //可变参数模板 ex: for armorDetector, mDest ==> std::tuple<SucceedAddress<armor_detect_available_atom>, SucceedAddress<image_frame_atom>>
    std::tuple<SucceedAddress<Succeed>...> mDest;

   /**
   * @brief				得到atom要发送的目标actor
   * @retrun			返回一个std::vector<std::pair<caf::actor_addr, GroupMask>>> 类型的值	
   */    
    template <typename Atom>
    const auto& getDest() {
        //从mDest中拿到要发送的actor的名称
        auto& dest = std::get<SucceedAddress<Atom>>(mDest).val;
   		//如果dest为std::variant中的std::vector<std::string>类型，则将其变为std::vector<std::pair<caf::actor_addr, GroupMask>>类型     
        if(dest.index() == 0)
            dest = detail::parseSucceed(this->system(), std::get<0>(dest));
        return std::get<1>(dest);
    }

protected:

   //在编译期，给定不同的参数，返回不同的类型
   //std::is_void_v<config>为true，返回类型char,否则返回Config
    std::conditional_t<std::is_void_v<Config>, char, Config> mConfig;
    
    //GroupMask 32位整型，哨兵两个云台需要辨别上下云台数据，引入该Groupmask机制进行区分
    GroupMask mGroupMask;

   /**
   * @brief				根据this指针生成Key
   * @param thisPointer self类型的this指针
   * @retrun			某个actor类实例化的对象对应的key， key=类型的hash_code 异或 类实例化后的对象this指针的值的结果	
   */
    template <typename Self>
    static Identifier generateKey(Self* thisPointer) {
        return { typeid(Self).hash_code() ^ reinterpret_cast<uintptr_t>(thisPointer) };
    }

public:
    /**
   * @brief				构造函数，读取配置文件，初始化actor
   						1.如果配置文件中，如果设置了group_mask则mGroupMask为group_mask,如果设置了						                group_id则mGroupMask为1<<group_id,，否则默认为1
   * @param base 		actor基类
   * @param config		配置文件
   */
    HubHelper(caf::actor_config& base, const HubConfig& config)
        : T{ base }, mDest{ SucceedAddress<Succeed>{ detail::parseSucceed(config, typeid(Succeed).name()) }... } {
        if constexpr(!std::is_void_v<Config>) {
            if(auto configValue = caf::get_as<Config>(config)) {
                mConfig = std::move(configValue.value());
            } else {
                logError("Bad config");
            }
        }

        const auto& dict = config.to_dictionary();
        if(const auto iter1 = dict->find("group_mask"); iter1 != dict->cend()) {
            mGroupMask = static_cast<uint32_t>(iter1->second.to_integer().value());
        } else if(const auto iter2 = dict->find("group_id"); iter2 != dict->cend()) {
            mGroupMask = 1U << static_cast<uint32_t>(iter2->second.to_integer().value());
        } else {
            mGroupMask = 1U;
        }
    }
    /**
   * @brief				给对应的actor发送atom，从而触发相应lambada函数调用
   * @param atom		需要发送的atom
   * @param args		其他参数
   */
    template <typename Atom, typename... Args>
    void sendAll(Atom atom, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, mask] : getDest<Atom>())
            this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }
    /**
   * @brief				当目标actor的mask和mask相同时， 给目标actor发送atom，从而触发相应lambada函数调用
   * @param atom		需要发送的atom
   * @param mask		目标acotr的mask
   * @param args		其他参数
   */
    template <typename Atom, typename... Args>
    void sendMasked(Atom atom, GroupMask mask, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, maskRhs] : getDest<Atom>())
            //检查目标acotr的mask是否和mask一致
            if(mask & maskRhs)
                this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }
};
```

## BlackBoard

+ `std::shared_mutex`
    + 用于多个读线程能够同时访问同一资源而不导致数据竞争，但只有一个写线程能访问的情形。

+ `std::lock_guard`
    + 根据RAII，将锁的持有视为资源，构造函数里调用`mtx.lock()`获取锁，则上锁；退出函数作用域时，调用`mtx.unlock()`析构释放锁，即解锁。

+ `std::any `和`std::any_cast<T>`
    + `std::any`可用于任何类型单个值的类型安全的容器, `std::any_case<T>`类型转换

+ `std::nullopt`
    + 空类类型，指示具有未初始化状态的`optional`类型

```c++
class BlackBoard final {
  	//存放大型结构体的Map, actor发送的数据都存放在这里, size_t为值类型的哈希编码和key异或的结果， 下面简称哈希码
    std::unordered_map<size_t, std::pair<std::shared_mutex, std::any>> mItems;
    //读写锁，访问mItems时上锁
    std::shared_mutex mMutex;
    
  /**
   * @brief				向黑板上插入值
   * @param hashValue 	哈希码
   * @param val 	  	需要插入的值	
   */
    void insertImpl(size_t hashValue, std::any val);
    
   /**
   * @brief 			根据哈希码从黑板上拿值
   * @param hashValue   值类型的哈希码
   * @return 	  		拿到的值	
   */
    std::pair<std::shared_mutex, std::any>* getImpl(size_t hashValue);

public:
    /**
   * @brief 			根据key从黑板上拿值
   * @param key   		每个actor类实例的对象对应的整形值
   * @return 	  		拿到的值	
   */
    template <typename T>
    std::optional<T> get(const Identifier key) {
        if(const auto ptr = getImpl(typeid(T).hash_code() ^ key.val)) {
            std::shared_lock guard{ ptr->first };
            return std::any_cast<T>(ptr->second);
        }
        return std::nullopt;
    }
    /**					
   * @brief 			更新黑板上的值
   * @param key   		每个actor？atom?对应的整形值?
   * @return 	  		key的值	
   */
    template <typename T>
    TypedIdentifier<T> updateSync(const Identifier key, T val) {
        const auto hashCode = typeid(T).hash_code() ^ key.val;
        if(const auto ptr = getImpl(hashCode)) {
            std::lock_guard guard{ ptr->first };
            ptr->second = std::move(val);
        } else
            insertImpl(hashCode, std::move(val));
        return { { key.val } };
    }

    static BlackBoard& instance();
};
```
# ACTOR_PROTOCOL_DEFINE 

+ **该宏定义atom与发atom时需要从`blackboard`上拿的对应的结构体类型**

+ 使用方法`ACTOR_PROTOCOL_DEFINE(atom, typename)`，`atom`为一个结构体类型，`typename`为与之对应的结构体类

    ```c++
    //该宏定义了一个模板类，该模板类相当于
    struct __ImplActorProtocol<atom, typename> final{
        static constexpr bool check() noexcept{
            return true;
        }
    }
    #define ACTOR_PROTOCOL_DEFINE(...)                  \
        template <>                                     \
        struct __ImplActorProtocol<__VA_ARGS__> final { \
            static constexpr bool check() noexcept {    \
                return true;                            \
            }                                           \
        }
    ```

# ACTOR_PROTOCOL_CHECK

+ 该宏检查`atom`与参数类型是否对应

```c++
//该宏调用 __ImplActorProtocol<Args...>:：check()进行判断
#define ACTOR_PROTOCOL_CHECK(...) static_assert(__impl_actor_protocol_call<__VA_ARGS__>(), "Mismatched protocol")

template <typename... Args>
constexpr bool __impl_actor_protocol_call() noexcept {
    return __ImplActorProtocol<Args...>::check();
}

//没有经过__ACTOR_PROTOCOL_DEFINE__定义的其他类型参数传入时，利用模板偏特化技术，会匹配到该类，从而返回false
template <typename... T>
struct __ImplActorProtocol final {
    static constexpr bool check() noexcept {
        return false;
    }
};
```

# ACTOR_EXCEPTION_PROBE

+ `__FILE__`: 当前源文件名
+ `__LINE_`: 当前程序行的行号
+ `__FUNCTION__`:当前函数的函数名
+ `feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)`
    + 该函数使能浮点数异常检查功能
    + `FE_DIVBYZERO `表示被0除的异常
    + `FE_INVALID`表示不合法的浮点运算
    + `FE_OVERFLOW`表示浮点数溢出

+ `std::uncaught_exceptions`

    + 检测当前线程中是否有活动异常对象，及异常已抛出或重新抛出，并且尚未输入匹配的`CATHCH`字句

+ `__builtin_trap`

    + 本质上通过执行非法命令来中止程序。

      ```css
    __builtin_trap function causes the program to exit abnormally. GCC implements this function by using a target-dependent mechanism (such as intentionally executing an illegal instruction) or by calling abort. The mechanism used may vary from release to release so you should not rely on any particular implementation.
      ```

```c++
//定义ExceptionProbe __probe{}类
#define ACTOR_EXCEPTION_PROBE()          \
    ExceptionProbe __probe {             \
        __FILE__, __FUNCTION__, __LINE__ \
    }

void installFPEProbe() {
#ifdef ARTINXHUB_DEBUG
#ifdef ARTINXHUB_WINDOWS
    _control87(_EM_DENORMAL | _EM_INEXACT | _EM_UNDERFLOW, _MCW_EM);
#else
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
#endif
}
void uninstallFPEProbe();

class ExceptionProbe final {
    const char* mFile;
    const char* mFunction;
    const uint32_t mLine;

    static constexpr auto highLatency = 50ms;
    Clock::time_point mStart;

public:
    ExceptionProbe(const char* file, const char* function, const uint32_t line)
        : mFile{ file }, mFunction{ function }, mLine{ line }, mStart{ Clock::now() } {
#ifdef ARTINXHUB_DEBUG
        installFPEProbe();
#endif
    }
    ExceptionProbe(const ExceptionProbe& rhs) = delete;
    ExceptionProbe& operator=(const ExceptionProbe& rhs) = delete;
    ExceptionProbe(ExceptionProbe&& rhs) = delete;
    ExceptionProbe& operator=(ExceptionProbe&& rhs) = delete;

    ~ExceptionProbe() {
#ifdef ARTINXHUB_DEBUG
        uninstallFPEProbe();

        if(std::uncaught_exceptions()) {
#ifdef ARTINXHUB_WINDOWS
            __debugbreak();
#else
            __builtin_trap();
#endif
        }
#else
        if(Clock::now() - mStart > highLatency) {
            logWarning(fmt::format("High latency detected {} {} {}", mFile, mFunction, mLine));
        }
#endif
    }
};
```



## Armor Detector

### 吉林大学2020方案

+ *Reference*：[GitHub - QunShanHe/JLURoboVision: Standard Vision Software of TARS-GO Team, Jilin University on RoboMaster 2020 Robotic Competition](https://github.com/QunShanHe/JLURoboVision)

#### 二值化方案
```C++
//RGB通道相减
enemyColor == RED:
    r - b > thereshold
enemyColor == BLUE:
    b - r > thereshold

//筛选结构化的元素、膨胀
kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3))
dilate(src, dist, kernel)
```
#### 灯条寻找方案
```C++
//寻找轮廓
findContours(contourImg, lightContours, CV_RETR_EXTERNAL，CV_CHAIN_APPROX_SIMPLE)

// 对找到的轮廓遍历进行筛选，筛选条件如下
// 1、轮廓点数 > 6
// 2、轮廓面积大于 min_area 阈值

//拟合椭圆
fitEllipse(lightContours)
//角度筛选，去掉角度偏大的灯条

//将灯条从左到右排序
```
#### 灯条匹配方案
 ```c++
 // 从左到右，为每个灯条编号， 并与其他灯条进行一次匹配判断，判断条件如下
// 1、角度差判断
// 2、两灯条中心连线与与水平线夹角
// 3、两灯条中心x方向差距比值（l_light_center.x - l_light_center.y) / mean_l_light_len)
// 4、两灯条中心y方向差距的比值
// 左右灯条长度差比值 (l_light.len - r_light.leng) / max(l_light_len, r_light_len)

//去除游离灯条导致错误识别的装甲板
//如果装甲板左右两边灯条编号一致，则比较两装甲板灯条中心连线与水平线的夹角，谁小，则去除另外一个。
 ```
### Artinx 2022视觉方案

#### 二值化方案

```c++
static bool isWhite(int32_t b, int32_t g, int32_t r) {
    return b + g + r > 520;
}

// for blue
 (!isWhite(b, g, r) && b > minB && g < maxG && r < maxR && b * 4 > g + r) ? 255 : 0

//for red
(!isWhite(b, g, r) && r > minR && b < maxB && g < maxG && r > b + g) ? 255 : 0
```

#### 灯条寻找方案
#### cv::minAreaRect旋转矩形定义测试代码
```C++
    void testMinAreaRect(){
        cv::Mat test_image(200, 200, CV_8UC3, cv::Scalar(0));
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(100, 100), cv::Size2f(100, 50), 180);

        //绘制旋转矩形
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++){
            cv::line(test_image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        cv::Mat binarySrc;
        cv::cvtColor(test_image, binarySrc, cv::COLOR_BGR2GRAY);
        cv::threshold(binarySrc, binarySrc, 0, 255, cv::THRESH_OTSU);

        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binarySrc, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> lights;
        for(auto& lightContour : contours) {
            auto rect = cv::minAreaRect(lightContour);
            logInfo(fmt::format("Rect height:{}, Rect width:{}, Rect Angle:{}", rect.size.height, rect.size.width, rect.angle));
            cv::Point2f rotateVertices[4];
            rect.points(rotateVertices);
            for (int i = 0; i < 4; i++){
                cv::circle(test_image, rotateVertices[i], 2, cv::Scalar(0, 255, 0), 2);
                cv::putText(test_image, std::to_string(i), rotateVertices[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
                logInfo(fmt::format("Vertices[{}]:({}, {})", i, rotateVertices[i].x, rotateVertices[i].y));
            }
        }

        debugView("RotateRect", test_image, [](auto){});
    }
```

```C++
//寻找轮廓
cv::findContours();

/*最小矩形拟合轮廓
注意opencv4返回的旋转矩形定义：
角度为水平顺时针旋转碰到的一条边(该边定义为宽)所转的角度，范围在(0, 90]

*/
rawRect = cv::minAreaRect(lightContour);


/*去除不符合条件的灯条，不符合条件的灯条情况如下:
1、max(高、宽) < 10 && min(宽、高) > 50
2、高 > 2 * 宽  && Rect.angle < maxLightAngle
3、高 > maxLightRectRatio * 宽 && Rect.width > 3
*/
```

#### 灯条匹配方案

```C++
//以两条灯条矩形拟合的最小外接矩形为装甲板的矩形
//规定长边为装甲板宽，短边如装甲板高
if(rect.size.width < rect.size.height) {  // rotate rect
    std::swap(rect.size.width, rect.size.height);
}

//diff =（rectRatio - ArmorRatio） / ArmorRatio
//par = abs(lhs.angle - rhs.angle)
/*去除不符合装甲板矩形，不符合的装甲板矩形情况如下:
1、矩形高度 < 3.0f
2、diff  > maxArmorRectRatio
3、两个灯条的中心线的连线与水平方向的夹角 > maxArmorAngle
4、如果两个灯条面积比 < 0.1f 并且左右两灯条矩形长宽的比率大的那个 < 0.5
5、左右两个灯条矩形的面积 > 0.5 * 装甲板矩形
6、左右两灯条的矩形角度与装甲板矩形的角度差 > minLightBaseAngle
7、两个灯条的的角度差 > maxParallelAngle
8、两个灯条的最大的高度 > 1.2 * 矩形高度
9、两个灯条最小高度 < minLightHeightRatio * rect.size.height
*/

//根据 diff + par 的值对匹配的灯条构成的矩形进行排序
//依次将矩形放入结果中，如果构成某个矩形的灯条已经被用过了，则丢弃掉。

//去除反射：由于灯光照在地面上可能导致地面上的灯光被误识别为装甲板，因此需去除

```



#### 深圳大学2019年方案

+ *Reference*：[GitHub - yarkable/RP_Infantry_Plus: RoboMaster2019 Infantry Vision OpenSource Code of Shenzhen University](https://github.com/yarkable/RP_Infantry_Plus)

## SolvePnP

+ *Reference*:[OpenCV: Perspective-n-Point (PnP) pose computation](https://docs.opencv.org/3.4/d5/d1f/calib3d_solvePnP.html)
+ *Reference*:https://www.cnblogs.com/singlex/category/911880.html
## Coordinate system regulation and coordinate transformation

### Coordinate system regulation

+ 坐标系统一采用右手系

+ 装甲板坐标以装甲板中心为原点，装甲板平板为xy平面。
+ 相机坐标系原点为相机光心的位置，沿着相机方向向后为z轴正，垂直相机向右为x轴正，垂直相机向上为y轴正。
+ 枪管坐标系的原点规定在枪管pitch轴旋转的两个支点的中点处，沿枪管朝前为z轴负，垂直枪管为向右为x轴正。
+ 机器人坐标系原点规定在底盘中心，向正右方为x轴正，正上方为y轴正，正后方为z轴正。

+ 世界坐标系原点和机器人坐标系原点重合

### Coordinate transformation

+ 从装甲板坐标系到相机坐标系

    通过`SolvePnP`能得到，从装甲板坐标系到相机坐标系的变化矩阵$^cT_w$

    取装甲板中心，即$X_w = 0，Y_w = 0, Z_w = 0$,则 $X_c = t_x, Y_c=t_y, Z_c=t_z$,由于`opencv solvePnP`中规定的y轴方向和z轴方向相反，所以y轴和z轴方向还需要做一个取负的运算。

+ 从相机坐标系到枪管坐标系

    由于相机安装会和枪管有一个固定的偏置，所以从相机坐标系到枪管坐标系需要有一个平移变换。

+ 从机器人坐标系从枪管坐标系

    利用`glm::lookAtRH`函数，以机器人坐标系为原点，去看枪管坐标系。

    *Reference*:[摄像机+LookAt矩阵+视角移动+欧拉角 - Garrett_Wale - 博客园 (cnblogs.com)](https://www.cnblogs.com/GarrettWale/p/11336589.html)
    
    ```c++
        /**					
       * @brief 				根据要变换到的坐标系的原点在当前坐标系的位置，和三个向量的方向在当前坐标系的位置来求得两个坐标系之间的变换
       * @param1 eye			要变换到的坐标系原点在当前坐标系的位置，
       							枪管坐标系原点在机器人坐标系（0.0, mConfig.headHeightOffset1, mConfig.headForwardOffset1）
       * @param center 	  	    要变换到的坐标系三个向量的方向，根据yaw角和pitch角推出，可以自己想一想怎么推出来的
       * @param up				上向量
       */
    const HeadInfo infoUp{ SynchronizedClock::instance().now(),
                                       decltype(HeadInfo::tfRobot2Gun){ glm::lookAtRH(
                                           glm::dvec3{ 0.0, mConfig.headHeightOffset1, mConfig.headForwardOffset1},
                                           glm::dvec3{ std::cos(static_cast<double>(fdb.yaw) + glm::half_pi<double>()) *
                                                           std::cos(static_cast<double>(fdb.pitch)),
                                                       mConfig.headHeightOffset1 + std::sin(static_cast<double>(fdb.pitch)),
                                                       mConfig.headForwardOffset1 - std::sin(static_cast<double>(fdb.yaw) + glm::half_pi<double>()) *
                                                           std::cos(static_cast<double>(fdb.pitch)) },
                                           glm::dvec3{ 0.0, 1.0, 0.0 }) },
    ```
    
    

## Angle Solver

+ 以车辆自身作为参考系

+ 考虑视觉算法处理的延迟、串口通信的延迟，云台电机执行动作的延迟以及子弹从拨弹轮到射出的延迟，这几类延迟之和设为$t_{f}$

    $V_0$: 子弹相对车速度    $V_{0h}$ : 子弹的相对车水平面方向速度 	$V_{0v}$:子弹竖直方向的净速度  $V_2$: 目标移动的速度

     $V_1:$ 车的速度   $S$: 目标到枪口水平面方向的距离      $t$: 子弹从枪口射出到命中障碍物的时间   $h$: 目标到枪口的竖直高度

+ 未知量有$\vec{V_{0h}}$  $V_{0v}$  $t$，其余量已知

经过固定延迟$t_f$后，目标移动到$\vec{X_f}$处

​					$\vec{S_f} = \vec{S} + (\vec{V_2}-\vec{V_1})t_f$

$V_{0v}t + \frac{1}{2}gt^2 = h$     												 --式一

$\vec{V_{0h}}t = \vec{S_f} + (\vec{V_2} - \vec{V_1})t$

$\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ 

$V_{0h}cos\theta t= S_{fx} + ({V_2} - {V_1})_xt$   						--式二

$V_{0h}sin\theta t= S_{fy} + ({V_2} - {V_1})_yt$							--式三

$V_0^2 = V_{0h}^2 + V_{0v}^2 $														--式四

联立式一、式二、式三、式四，利用`matlab`可解得一个关于`t`的四次方程， `matlab`脚本如下，图片不方便插入，想看结果自己运行查看一下。

```matlab
syms V0v V0hx V0hy t; 
syms Sx Sy Vx Vy h g V0;
S1 = V0v*t + 1/2 * g * t * t - h;
S2 = V0hx*t - Sx  - Vx * t;
S3 = V0hy*t - Sy  - Vy * t;
S4 = V0 * V0 - V0v * V0v - V0hx * V0hx - V0hy * V0hy;
[V0v, V0hx, V0hy, t] = solve(S1, S2, S3, S4, V0v, V0hx, V0hy, t)
```

解得t后，其他量可简单求之，最后

```c++
//由于机器人的yaw轴的零点在视觉定义的坐标系的pi/2处，所以求得的pitch角要减pi/2
double pitchAngle = std::asin(verticalSpeed / bulletSpeed);
double yawAngle = std::atan2(horizontalSpeedY, horizontalSpeedX) - glm::half_pi<double>();
```

# Predictor

## 开源方案

+ *Reference*：[freezing00/Baldr: 本项目为桂林电子科技大学Evolution战队2021赛季常规机器人视觉项目 (github.com)](https://github.com/freezing00/Baldr)
+ *Reference*：[WMJ2021/libControl/Predict at master · NZqian/WMJ2021 (github.com)](https://github.com/NZqian/WMJ2021/tree/master/libControl/Predict)

## Sentry actor workflow

### `camera_up `和`camera_down`

+ 初始化由于类实例化的对象地址不同，所以`mkey`值不相同，对应的在`blackboard`上的`CameraFrame`的哈希值不同。
+ `mGroup`未设置，都为1。

+ `sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(frameData)))`。

### `serial`

+ `mGroup`未设置，为1。
+ `sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, tfGround2Robot))` 发送姿态信息
+ `sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoUp))`。发送上云台枪管坐标系到机器人坐标变换矩阵信息。
+ ` sendMasked(update_head_atom_v, 2U, 2U, BlackBoard::instance().updateSync(Identifier{ mKey.val ^ 0xffffffff }, infoDown))`;发送下云台枪管坐标系到机器人坐标系的变换矩阵信息。

+ 接收 `[this](set_target_info_atom, GroupMask mask, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire)`    `mGroupMask == 1U` 为上云台数据，否则为下云台数据。

### `detector_up`和`detector_down`

+ 接收 `image_frame_atom` ，发送`sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)))`
+ config 文件中配置了`detector_up`和`detector_down`分别接收`camera_up`和`camera_down`的`atom`和`key`。
+ `armor_detector_up`和`aromor_detector_down`

+ 接受 `car_detector_available_atom`, 发送`sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)))`

### `armor_locator_up` 和`armor_locator_down`

+ 在config文件中`armor_loctor_up` 的`group_id`为0， `armor_locator_down ` 的 `group_id`为 1。

+ 接收`armor_detect_available_atom`, 发送`sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)))`
+ 接收 `(update_head_atom, GroupMask, Identifier key)`, 更新`actor`的 `mHeadKey`

### `strategy`

+ 接收`(detect_available_atom, GroupMask mask, Identifier key)`

+ 如果`selected`有数据，` (mask == 1U ? mLastSelected1 : mLastSelected2) = selected`;

    如果`selected`没有数据，`selected = (mask == 1U ? mLastSelected2 : mLastSelected1)`,指从另外一个云台拿数据。

+ 发送`sendMasked(set_target_atom_v, mask, BlackBoard::instance().updateSync<SelectedTarget>(Identifier{mKey.val ^ mask}, selected));`，注意`sendMasked`指定了接收对象
+ 接收`(update_head_atom, GroupMask mask, Identifier key)`,更新`mHead1`和`mHead2`的数据。

### `angleSolve_up` 和`angleSolve_down`

+  接收`(set_target_atom, Identifier key)`
+ 发送 `sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle, pitchAngle, isFire)`
+ 接收`(update_head_atom, GroupMask, Identifier key)`, 更新`mHeadKey`
+ 接收 `(update_posture_atom, Identifier key)`, 更新`mIMUKey`
