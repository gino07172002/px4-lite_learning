#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <mutex>


// ------------------------------------------------------------
// main
// ------------------------------------------------------------
class Topic
{
public:
    void publish(int newData)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        data=newData;
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
    int data=0;
    int generation=0;
    mutable std::mutex _mutex;
};

class Subscription
{
public:
    Subscription()=default;
    void sub(Topic& topic)
    {
        _topic=&topic;
    }
    bool copy(int& out)
    {
        if(_topic==nullptr)
        {
            return false;

        }
        else
        {
            out=_topic->getData();
            _lastGeneration=_topic->getGeneration();
            return true;
        }
    }
    bool updated()
    {
        if(_topic==nullptr)
            return false;

        if(_lastGeneration==_topic->getGeneration())
        {
            return false;
        }
        else
        {
            return true;
        }
    }

private:

    int _lastGeneration=0;
    Topic* _topic=nullptr;
};

class Publisher
{
public:
    Publisher()=default;
    void bind(Topic& topic)
    {
         _topic=&topic;
    }
    void publish(int data)
    {
        if(_topic!=nullptr)
        {
            _topic->publish(data);
        }
    }

private:
    Topic* _topic=nullptr;
};
int main()
{
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

    if(sub.updated())
    {
        std::cout<<" sub has new data! "<<std::endl;
    }
    else
    {
        std::cout<<" sub has no new data"<<std::endl;
    }
    int value=0;
    sub.copy(value);
    std::cout<<" sub value = "<<value<<std::endl;
    if(sub.updated())
    {
        std::cout<<" sub has new data! "<<std::endl;
    }
    else
    {
        std::cout<<" sub has no new data"<<std::endl;
    }
    sub.copy(value);
    std::cout<<" sub value = "<<value<<std::endl;


    pub.publish(100);

    std::cout << sub.updated() << '\n';
    std::cout << sub2.updated() << '\n';

    sub.copy(value);

    std::cout << sub.updated() << '\n';
    std::cout << sub2.updated() << '\n';

    std::cout<<" hi "<<std::endl;
    return EXIT_SUCCESS;
}
/*
[使用者提問]:
build的期間
我把binary清掉你重編可以查到問題嗎
還是當掉

[其他備忘/筆記]:
接下來你要在頁面上把每個協議都串過,並且確實看到前端有相關的串接欄位可供設定,且盡量讓有限的設定是以下拉選單呈現而不是讓使用者自己填(ex: opcua的id其形態是有限的(string,數字等)),然後開始review並呼叫codex luna max model進行review,
*/
/*
[執行結果紀錄 / Log]:
 sub has new data! 
 sub value = 6
 sub has no new data
 sub value = 6
1
1
0
1
 hi 

[使用者問題]:
你看我現在寫這樣怎麼樣
*/