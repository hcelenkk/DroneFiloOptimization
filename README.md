Drone Teslimat Optimizasyonu
Bu proje, drone'lar kullanılarak teslimat rotalarının optimizasyonunu gerçekleştiren bir yazılımdır. A* arama algoritması, CSP çözücüleri ve Genetik Algoritmalar (GA) gibi yöntemler kullanılarak drone rotaları, teslimat noktaları ve uçuş yasağı bölgeleri dikkate alınarak optimize edilir. Proje, performans analizi ve görselleştirme özellikleriyle desteklenmiştir.
Özellikler

A Algoritması: En kısa rotaları bulmak için kapasite kontrolü ile optimize edilmiş A algoritması.
CSP Çözücü: Her drone'un aynı anda yalnızca bir paket taşımasını sağlayan kısıtlı memnuniyet çözücüsü.
Genetik Algoritma: Teslimat rotalarını optimize etmek için genetik algoritma tabanlı çözüm.
Görselleştirme: Matplotlib ile drone rotalarının ve uçuş yasağı bölgelerinin harita üzerinde görselleştirilmesi.
Performans Analizi: Farklı senaryolar için algoritma performanslarının karşılaştırılması.

Gereksinimler

Python 3.8 veya üstü
Aşağıdaki Python paketleri:
matplotlib
numpy
asyncio (Pyodide ile uyumlu)
typing

Kurulum

Projeyi klonlayın:
git clone https://github.com/hcelenkk/DroneFiloOptimization.git
cd DroneFiloOptimization

Sanal ortamı oluşturun ve etkinleştirin:
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows

Bağımlılıkları yükleyin:
pip install -r requirements.txt

Projeyi çalıştırın:
python main.py

Çıktılar:

Performans metrikleri (çalışma süreleri, tamamlanma oranları, enerji tüketimi).
Görselleştirilmiş rotalar (output/scenario1_routes.png ve output/scenario2_routes.png dosyaları).

Dosya Yapısı
DroneFiloOptimization/
├── output/ #Görselleştirilen rotalar (output/scenario1_routes.png ve output/scenario2_routes.png dosyaları)
├── src/
│   ├── algorithms/         # Algoritma implementasyonları (a_star.py, csp.py, genetic_algorithm.py)
│   ├── models/            # Model tanımlar (drone.py, delivery_point.py, no_fly_zone.py)
│   ├── utils/             # Yardımcı fonksiyonlar ve görselleştirme (graph.py, visualization.py, data_generator.py)
├── main.py  # Ana script
├── requirements.txt        # Bağımlılıklar
├── README.md              # Bu dosya

