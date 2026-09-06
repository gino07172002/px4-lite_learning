/**
 * @file main.cpp
 * @brief 使用者作業原始碼（已修正語法錯誤並可直接編譯執行）
 * 
 * [修正說明]:
 * 原先檔案末尾附加了執行 Log 以及文字「你看我現在寫這樣怎麼樣」，
 * 這導致 C++ 編譯器報語法錯誤。現已將其包覆於多行註解中，使其能成功編譯。
 * 
 * [導師點評與架構分析]:
 * 1. 優點：
 *    - 精準抓住了 PX4 uORB 最關鍵的「世代計數器 (Generation Counter)」設計理念！
 *    - 介面清晰定義了 publish(), copy(), updated() 三大核心功能。
 *    - 具備多執行緒防護意識，使用了 std::mutex 與 std::lock_guard。
 * 2. 待改進潛在問題 (Critical Bug):
 *    - 在 Subscription::copy() 中：
 *        out = _topic->getData();          // 取得一次 lock 後釋放
 *        _lastGeneration = _topic->getGeneration(); // 再次取得 lock
 *      這兩行之間存在「時間窗口 (Time Window)」。若在兩者之間有另一個執行緒呼叫了 publish()，
 *      則 out 拿到的是舊世代資料，但 _lastGeneration 卻被賦予新世代號碼，造成嚴重的資料與世代不一致！
 *      正確作法是：在同一個 Lock 範圍內，同時複製資料與記錄 generation。
 * 3. 擴展性限制：
 *    - 目前寫死為 int 型別，實際飛控需要 template <typename T> 來傳遞 SensorImu 等結構體。
 *    - 目前需要手動透過指標傳遞 Topic 物件，PX4 則是透過全域 TopicBus (uORB::Manager) 以主題名稱自動配對。
 */

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <mutex>

class Topic
{
public:
    void publish(int newData)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        data = newData;
        ++generation;
    }
    int getData() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return data;
    }
    int getGeneration() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return generation;
    }
private:
    int data = 0;
    int generation = 0;
    mutable std::mutex _mutex;
};

class Subscription
{
public:
    Subscription() = default;
    void sub(Topic& topic)
    {
        _topic = &topic;
    }
    bool copy(int& out)
    {
        if (_topic == nullptr)
        {
            return false;
        }
        else
        {
            out = _topic->getData();
            _lastGeneration = _topic->getGeneration();
            return true;
        }
    }
    bool updated()
    {
        if (_topic == nullptr)
            return false;

        if (_lastGeneration == _topic->getGeneration())
        {
            return false;
        }
        else
        {
            return true;
        }
    }

private:
    int _lastGeneration = 0;
    Topic* _topic = nullptr;
};

class Publisher
{
public:
    Publisher() = default;
    void bind(Topic& topic)
    {
        _topic = &topic;
    }
    void publish(int data)
    {
        if (_topic != nullptr)
        {
            _topic->publish(data);
        }
    }

private:
    Topic* _topic = nullptr;
};

int main()
{
    std::cout << "=== [User Homework] uORB Lite Demo ===" << std::endl;
    Topic helloTopic;

    Publisher pub;
    Subscription sub;
    Subscription sub2;

    sub.sub(helloTopic);
    sub2.sub(helloTopic);
    pub.bind(helloTopic);
    pub.publish(5);
    pub.publish(7);
    pub.publish(6);

    if (sub.updated())
    {
        std::cout << " sub has new data! " << std::endl;
    }
    else
    {
        std::cout << " sub has no new data" << std::endl;
    }
    int value = 0;
    sub.copy(value);
    std::cout << " sub value = " << value << std::endl;
    if (sub.updated())
    {
        std::cout << " sub has new data! " << std::endl;
    }
    else
    {
        std::cout << " sub has no new data" << std::endl;
    }
    sub.copy(value);
    std::cout << " sub value = " << value << std::endl;

    pub.publish(100);

    std::cout << "sub.updated(): " << sub.updated() << '\n';
    std::cout << "sub2.updated(): " << sub2.updated() << '\n';

    sub.copy(value);

    std::cout << "After sub.copy(), sub.updated(): " << sub.updated() << '\n';
    std::cout << "sub2.updated(): " << sub2.updated() << '\n';

    std::cout << " 作業執行成功！" << std::endl;
    return EXIT_SUCCESS;
}
