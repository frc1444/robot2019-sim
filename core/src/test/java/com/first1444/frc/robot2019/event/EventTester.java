package com.first1444.frc.robot2019.event;

import java.util.Scanner;

public class EventTester {
	public static void main(String[] args){
		try(TCPEventSender sender = new TCPEventSender(5809)){
			Scanner scanner = new Scanner(System.in);
			while(scanner.hasNext()){
				final String data = scanner.nextLine();
				System.out.println("===We will send: '" + data + "'===");
				sender.sendEvent(data);
				System.out.println("===Sent!===");
			}
		}
	}
}
