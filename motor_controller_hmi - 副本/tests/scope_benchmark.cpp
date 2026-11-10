#include "scope.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <cmath>
#include <iostream>
using namespace mt;
int main(int argc, char **argv) {
    QApplication app(argc, argv);
    QJsonArray results;
    int channels = app.arguments().contains("--normal") ? normalChannelCount : channelCount;
    std::vector<size_t> sizes = {120000, 1000000};
    if (app.arguments().contains("--large"))
        sizes = {10000000};
    for (size_t count : sizes) {
        Sampling sampling(std::min<size_t>(10000000, count + 2000));
        auto append = [&](size_t i) {
            Sample s;
            s.time = i * .001;
            for (int c = 0; c < channels; ++c)
                s.values[c] = std::sin(i * (.1 + c * .017)) + (i % 997 == 0 ? 2 : 0);
            sampling.append(s);
        };
        for (size_t i = 0; i < count; ++i)
            append(i);
        Scope scope;
        scope.resize(1200, 650);
        scope.visible.fill(false);
        for (int c = 0; c < channels; ++c) scope.visible[c] = true;
        if (app.arguments().contains("--debug")) {
            scope.debugProfile = true;
            scope.aliases = debugChannelNames(1);
        }
        scope.bind(&sampling);
        QImage image(scope.size(), QImage::Format_RGB32);
        size_t seq = count;
        for (double seconds : {5.0, double(count) / 1000}) {
            scope.windowSeconds = seconds;
            std::vector<double> times;
            for (int frame = 0; frame < 40; ++frame) {
                for (int j = 0; j < 33; ++j)
                    append(seq++);
                QElapsedTimer timer;
                timer.start();
                scope.refresh();
                scope.render(&image);
                if (frame >= 5)
                    times.push_back(timer.nsecsElapsed() / 1e6);
            }
            std::sort(times.begin(), times.end());
            double sum = 0;
            for (double t : times)
                sum += t;
            results.append(QJsonObject{{"points", double(count)}, {"channels", channels},
                                       {"window_s", seconds},
                                       {"mean_ms", sum / times.size()},
                                       {"p95_ms", times[size_t(times.size() * .95)]}});
        }
    }
    std::cout << QJsonDocument(results).toJson().constData();
}
