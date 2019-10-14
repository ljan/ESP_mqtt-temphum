#ifdef DEBUG
 #define dbprint(x)  Serial.print(x)
 #define dbprintln(x)  Serial.println(x)
 #define dbserialbegin(x) Serial.begin(x);
#else
 #define dbprint(x)
 #define dbprintln(x)
 #define dbserialbegin(x)
#endif
