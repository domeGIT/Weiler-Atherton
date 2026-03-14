#include "SFML/Graphics.hpp"
#include <iostream>
#include <optional>
#include <cmath>
#include <vector>
#include <list>
#include <algorithm>
#include <fstream>
#include <sstream>


#define VERTEX 0
#define INTERSECTION 1
#define EPS 1e-6f

struct Point {
    float x, y;

    bool operator==(const Point& other) const {
        return std::abs(x - other.x) < EPS && std::abs(y - other.y) < EPS;
    }

};


// The main structure we use to implement the algorithm
struct Node {
    Point point;
    int type;      // is it vertex or intersection
    int subject_edge;       // Index in subject polygon edge
    int clip_edge;         // Index in clip polygon edge            
    bool isEntry;        // Entry/Exit flag
 };

class Polygon {
public:
    std::vector<Point> vertices;

    void draw(sf::RenderWindow& window, sf::Color color) const {
        if (vertices.empty()) return;

        std::vector<sf::Vertex> line;
        for (const auto& point : vertices) {
            line.push_back(sf::Vertex{ { point.x, point.y }, color });
        }
        if (vertices.size() > 2) {
            line.push_back(sf::Vertex{ {vertices[0].x, vertices[0].y}, color});
        }

        if (!line.empty()) {
            window.draw(line.data(), line.size(), sf::PrimitiveType::LineStrip);
        }
    }

    void drawPoints(sf::RenderWindow& window, sf::Color color, float radius = 5.0f) const {
        for (const auto& point : vertices) {
            sf::CircleShape circle(radius);
            circle.setFillColor(color);
            circle.setPosition({ point.x - radius, point.y - radius });
            window.draw(circle);
        }
    }
};


bool loadPolygonFromFile(const std::string& filename, Polygon& polygon) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file '" << filename << "'\n";
        return false;
    }

    polygon.vertices.clear();

    // Skip comment lines (starting with #)
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        float x, y;

        if (iss >> x >> y) {
            polygon.vertices.push_back({ x, y });
        }

    }

    file.close();

    if (polygon.vertices.size() < 3) {
        std::cerr << "Error: Polygon must have at least 3 vertices\n";
        return false;
    }

    std::cout << "Loaded " << polygon.vertices.size() << " vertices from '" << filename << "'\n";
    return true;
}




// Check if point is inside polygon
bool isPointInside(const Point& point, const std::vector<Point>& polygon) {
    bool inside = false;
    int n = polygon.size();

    for (int i = 0; i < n; i++) {
        const Point& p1 = polygon[i];
        const Point& p2 = polygon[(i + 1) % n];

        if (std::max(p1.y, p2.y) >= point.y && std::min(p1.y, p2.y) < point.y) {
            float x_intercept;
            if (p1.x == p2.x) {
                x_intercept = p1.x;
            }
            else {
                x_intercept = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            }

            if (x_intercept > point.x)
                inside = !inside;
        }
    }

    return inside;
}

bool isPointOnSegment(const Point& a, const Point& b, const Point& c) {
    float minx = std::min(a.x, b.x), maxx = std::max(a.x, b.x);
    float miny = std::min(a.y, b.y), maxy = std::max(a.y, b.y);

    if (c.x >= minx - EPS && c.x <= maxx + EPS &&
        c.y >= miny - EPS && c.y <= maxy + EPS)
        return true;
    return false;
}

//Calculate orientation
int orientation(const Point& a, const Point& b, const Point& c) {
    float r = (b.y - a.y) * (c.x - a.x) - (c.y - a.y) * (b.x - a.x);

    if (r < -EPS) return -1;
    if (r > EPS) return 1;
    return 0;
}

//Check if segments intersect
bool segmentsIntersect(const Point& A1, const Point& A2, const Point& B1, const Point& B2) {
    int o1 = orientation(B1, B2, A1);
    int o2 = orientation(B1, B2, A2);
    int o3 = orientation(A1, A2, B1);
    int o4 = orientation(A1, A2, B2);

    // General intersection
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
        ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))
        return true;

    // Special cases - NOT USED HERE
    if (o1 == 0 && isPointOnSegment(B1, B2, A1))
        return true;
    if (o2 == 0 && isPointOnSegment(B1, B2, A2))
        return true;
    if (o3 == 0 && isPointOnSegment(A1, A2, B1))
        return true;
    if (o4 == 0 && isPointOnSegment(A1, A2, B2))
        return true;

    return false;
}

//Calculate the intersection using the Cramer rule
Point getIntersection(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
    Point p;

    float A1 = p2.y - p1.y;
    float B1 = p1.x - p2.x;
    float C1 = A1 * p1.x + B1 * p1.y;

    float A2 = p4.y - p3.y;
    float B2 = p3.x - p4.x;
    float C2 = A2 * p3.x + B2 * p3.y;

    float D = A1 * B2 - A2 * B1;

    p.x = (C1 * B2 - C2 * B1) / D;
    p.y = (A1 * C2 - A2 * C1) / D;

    return p;
}


// Find all the intersection points between polygons
void FindIntersections(const Polygon& subject, const Polygon& clip,
    std::vector<Node>& intersections) {
    int subjSize = subject.vertices.size();
    int clipSize = clip.vertices.size();

    for (int i = 0; i < subjSize; i++) {
        const Point& p1 = subject.vertices[i];
        const Point& p2 = subject.vertices[(i + 1) % subjSize];

        for (int j = 0; j < clipSize; j++) {
            const Point& p3 = clip.vertices[j];
            const Point& p4 = clip.vertices[(j + 1) % clipSize];

            if (segmentsIntersect(p1, p2, p3, p4)) {
                Node intersection;
                intersection.point = getIntersection(p1, p2, p3, p4);
                intersection.type = INTERSECTION;
                intersection.subject_edge = i;  // Index in subject polygon
                intersection.clip_edge = j;   // Index in clip polygon             
                intersections.push_back(intersection);
            }
        }
    }
}

// Generate vector with original vertices + intersections in order
void addIntersections(const Polygon& polygon, std::vector<Node>& intersections,
    std::vector<Node>& polygon_extended, int index) {
    int size = polygon.vertices.size();

    for (int i = 0; i < size; i++) {
        // Add original vertex
        Node node;
        node.type = VERTEX;
        node.point = polygon.vertices[i];
        polygon_extended.push_back(node);

        auto distance = [](const Point& a, const Point& b)-> float {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return dx * dx + dy * dy;
            };
        
        // Find all intersections on this edge
        std::vector<Node> edgeIntersections;
        for (auto& intersection : intersections) {
            if ((index == 0 && i == intersection.subject_edge) ||
                (index == 1 && i == intersection.clip_edge)) {
                edgeIntersections.push_back(intersection);
            }
        }

        // Sort intersections along the edge
        std::sort(edgeIntersections.begin(),edgeIntersections.end(),[&](const Node& a, const Node& b) {
            return distance(a.point,node.point) < distance(b.point, node.point);
            });

        // Add sorted intersections
        for (auto& intersect : edgeIntersections) {
            polygon_extended.push_back(intersect);
        }
    }
}

// Determine if intersections are entries or exits 
void classifyIntersections(std::vector<Node>& subject_extended, const Polygon& clip) {
    bool helperFlag = false;

    for (auto& subject_Node : subject_extended) {
        if (subject_Node.type == VERTEX) {
            // Original vertex - test if inside clip polygon
            helperFlag = isPointInside(subject_Node.point, clip.vertices);
        }
        else {
            // Intersection point - toggles in/out
            helperFlag = !helperFlag;
            subject_Node.isEntry = helperFlag;
        }
    }
}

// Copy in/out flags to clip polygon intersections
void matchIntersectionFlags(std::vector<Node>& clip_extended,
    std::vector<Node>& subject_extended) {
    for (auto& clip_Node : clip_extended) {
        if (clip_Node.type == VERTEX) 
            continue;

        for (auto& subject_Node : subject_extended) {
            if (subject_Node.type == VERTEX) 
                continue;

              if (clip_Node.point == subject_Node.point){
                clip_Node.isEntry = subject_Node.isEntry;
                break;
            }
        }
    }
}

// Generate the clipped polygons
std::vector<Polygon> createPolygons(std::vector<Node>& subject_extended,
    std::vector<Node>& clip_extended) {
    std::vector<Polygon> result;
    std::vector<bool> visited(subject_extended.size(), false);

    std::cout << "=================WEILER-ATHERTON=================\n";
    int number_of_polygon = 0;

    // Find first entry point
    for (size_t i = 0; i < subject_extended.size(); i++) {
        if (subject_extended[i].type == INTERSECTION &&
            subject_extended[i].isEntry &&
            !visited[i]) {

            Polygon currentPoly;
            size_t currentIdx = i;
            bool inSubject = true;
            Point startPoint = subject_extended[i].point;

            std::cout << "Starting new polygon at (" << startPoint.x << ", " << startPoint.y << ")\n";

            do {
                if (inSubject) {
                    // Traverse Subject polygon
                    currentPoly.vertices.push_back(subject_extended[currentIdx].point);

                    if (subject_extended[currentIdx].type == INTERSECTION) {
                        visited[currentIdx] = true;
                    }

                    // Check if this is an exit point
                    if (subject_extended[currentIdx].type == INTERSECTION &&
                        !subject_extended[currentIdx].isEntry) {

                        // Find matching point in clip polygon
                        bool found = false;
                        for (size_t j = 0; j < clip_extended.size(); j++) {
                            if (clip_extended[j].type == INTERSECTION &&
                                clip_extended[j].point==subject_extended[currentIdx].point) {
                                int previousIdx = currentIdx;
                                currentIdx = j;
                                inSubject = false;
                                found = true;
                                std::cout << "  [SUBJ->CLIP] Switch at (" << subject_extended[previousIdx].point.x
                                    << ", " << subject_extended[previousIdx].point.y << ")\n";
                                break;
                            }
                        }
                        if (!found) {
                            std::cerr << "ERROR: Could not find matching clip intersection!\n";
                            break;
                        }
                        continue;  // Continue loop without incrementing
                    }

                    currentIdx = (currentIdx + 1) % subject_extended.size();
                }
                else {
                    // Traverse Clip polygon
                    currentPoly.vertices.push_back(clip_extended[currentIdx].point);

                    // Check if this is an entry point back to subject
                    if (clip_extended[currentIdx].type == INTERSECTION &&
                        clip_extended[currentIdx].isEntry) {

                        // Find matching point in subject polygon
                        bool found = false;
                        for (size_t j = 0; j < subject_extended.size(); j++) {
                            if (subject_extended[j].type == INTERSECTION &&
                                subject_extended[j].point==clip_extended[currentIdx].point) {
                                int previousIdx = currentIdx;
                                currentIdx = j;
                                inSubject = true;
                                found = true;
                                std::cout << "  [CLIP->SUBJ] Switch at (" << clip_extended[previousIdx].point.x
                                    << ", " << clip_extended[previousIdx].point.y << ")\n";
                                break;
                            }
                        }
                        if (!found) {
                            std::cerr << "ERROR: Could not find matching subject intersection!\n";
                            break;
                        }
                        continue;  // Continue loop without incrementing
                    }

                    currentIdx = (currentIdx + 1) % clip_extended.size();
                }

                //  Check if we returned to the starting point
                Point currentPoint = inSubject ? subject_extended[currentIdx].point
                    : clip_extended[currentIdx].point;

                if (currentPoint==startPoint) {
                    std::cout << "  [COMPLETE] Returned to start point!\n";
                    break;
                }

            } while (true);  // Infinite loop, broken by point match

            // POLYGON CLEANUP
            if (currentPoly.vertices.size() >= 3) {
                // Remove consecutive duplicates
                std::vector<Point> cleaned;
                for (size_t k = 0; k < currentPoly.vertices.size(); k++) {
                    if (cleaned.empty() || !(cleaned.back()==currentPoly.vertices[k])) {
                        cleaned.push_back(currentPoly.vertices[k]);
                    }
                }

                // Remove duplicate last point if it matches first
                if (cleaned.size() >= 2 && cleaned.front()==cleaned.back()) {
                    cleaned.pop_back();
                }

                // Only add if we have at least 3 unique vertices
                if (cleaned.size() >= 3) {
                    currentPoly.vertices = cleaned;
                    result.push_back(currentPoly);

                    std::cout << "Polygon " << ++number_of_polygon << " has " << cleaned.size() << " vertices\n";
                    for (const auto& v : cleaned) {
                        std::cout << "    (" << v.x << "," << v.y << ")\n";
                    }
                    std::cout << "-------------------------------------------------\n";
                }
            }
        }
    }

    std::cout << "Number of polygons: " << number_of_polygon << "\n";
    return result;
}

// Main Weiler-Atherton clipping function
std::vector<Polygon> weilerAthertonClip(const Polygon& subject, const Polygon& clip) {
    std::vector<Node> intersections, subject_extended, clip_extented;

    // Step 1: Find all intersections
    FindIntersections(subject, clip, intersections);


    //Special case: Polygons don't intersect
    if (intersections.empty())
    {
        std::vector<Polygon> result;
        if (isPointInside(subject.vertices[0], clip.vertices))
        {   
            std::cout << "The whole polygon is inside the clipping area!!!\n";
            result.push_back(subject);
        }
        else
        {
            std::cout << "The whole polygon is outside the clipping area!!!\n";
        }
        return result;
    }

    // Step 2: Add intersection points to polygon vectors
    addIntersections(subject, intersections, subject_extended, 0);  // 0 for subject
    addIntersections(clip, intersections, clip_extented, 1);      // 1 for clip

    // Step 3: Classify entry/exit points
    classifyIntersections(subject_extended, clip);

    // Step 4: Propagate flags to clip polygon
    matchIntersectionFlags(clip_extented, subject_extended);


    // Step 5: Generate clipped polygons
    return createPolygons(subject_extended, clip_extented);
}

int main() {
    // Create window
    sf::RenderWindow window(sf::VideoMode({ 1000, 800 }), "Weiler-Atherton Polygon Clipping");

    // Define subject polygon (red)
    Polygon subject;
    if (!loadPolygonFromFile("polygon.txt", subject)) {
        std::cerr << "Failed to load subject polygon, using default\n";
        subject.vertices = {
            {50.f, 50.f}, {450.f, 100.f}, {350.f, 150.f}, {150.f, 200.f},
            {250.f, 250.f}, {450.f, 300.f}, {350.f, 350.f}, {150.f, 400.f}, {50.f, 450.f}
        };
    }


    // Define clip polygon (blue)
    Polygon clip;
    if (!loadPolygonFromFile("clip.txt", clip)) {
        std::cerr << "Failed to load clipping polygon, using default\n";
        clip.vertices = {
            {100.f, 100.f}, {100.f, 400.f}, {400.f, 400.f}, {400.f, 100.f}
        };
    }

    // Perform clipping
    std::vector<Polygon> result = weilerAthertonClip(subject, clip);

    // Main drawing loop
    while (window.isOpen()) {
        // Event handling
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
            else if (event->is<sf::Event::KeyPressed>()) {
                if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Escape)
                    window.close();
            }
        }

        window.clear(sf::Color(20, 20 , 20));  // Dark  background

        // Draw original polygons with transparency
        subject.draw(window, sf::Color(255, 100, 100, 100));  //  Red
        clip.draw(window, sf::Color(100, 100, 255, 100));     //  Blue

        // Draw vertices
        subject.drawPoints(window, sf::Color::Red, 5);
        clip.drawPoints(window, sf::Color::Blue, 5);

        // Draw result polygons in green
        sf::Color resultColors[] = {
            sf::Color(0, 255, 0),      // Green
            sf::Color(255, 255, 0),     // Yellow
            sf::Color(255, 0, 255),     // Magenta
            sf::Color(0, 255, 255)      // Cyan
        };

        for (size_t i = 0; i < result.size(); i++) {
            result[i].draw(window, resultColors[i % 4]);
            result[i].drawPoints(window, resultColors[i % 4], 5);

            // Draw thicker lines for result
            std::vector<sf::Vertex> thickLine;
            const auto& verts = result[i].vertices;
            for (size_t j = 0; j < verts.size(); j++) {
                thickLine.push_back(sf::Vertex{{ verts[j].x, verts[j].y },
                    resultColors[i % 4]});
            }
            if (!verts.empty()) {
                thickLine.push_back(sf::Vertex{ {verts[0].x, verts[0].y},
                    resultColors[i % 4]});
            }
            window.draw(thickLine.data(), thickLine.size(), sf::PrimitiveType::LineStrip);
        }

        window.display();
    }

    return 0;
}