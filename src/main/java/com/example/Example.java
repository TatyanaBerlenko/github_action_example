package com.example;

public class Example {

    private static final String GREETING = "Hello, World!";

    public static void main(String[] args) {
        System.out.println(GREETING);
        int a = 5;
        int b = 5;
        if (a == b) {
            System.out.println("a and b are equal");
        }
    }

    public String getGreeting() {
        return GREETING;
    }
}
