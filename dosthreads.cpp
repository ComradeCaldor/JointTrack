#include<iostream> 
#include<thread> //Файл в котором определен класс thread 
using namespace std; 
 
void printStr(char * str) { 
    cout << str << '\n'; 
} 
 
void printArray(int a[],const int len) { 
    for (int i = 0; i < len; i++) { 
        cout << a[i] << ' '; 
    } 
} 
 
int main() { 
    char* str = "thread function with parametrs"; 
    const int len = 8; 
    int arr[len] = {12, 45, -34, 57, 678, 89, 0, 1}; 
    // Передаем параметр функции во время инициализации 
    thread func_thread(printStr, str); 
    // Параметров может быть много 
    thread func_thread2(printArray, arr, len);  
    if (func_thread.joinable()) func_thread.join(); 
    if (func_thread2.joinable()) func_thread2.join(); 
    return 0; 
}
