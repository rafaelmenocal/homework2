Class Point_Cloud {
    private:
    vector<Vector2f> points;

    public:
    Point_Cloud() {
        points = vector<Vector2f>(); //empty vector
    };

    Point_Cloud(vector<float>& ranges, float_t min_angle, float_t max_angle, float_t num_angles) {
        points = vector<Vector2f>(num_angles);
        float_t angle_inc = (max_angle - min_angle) / num_angles;
        for (int i = 0; i < num_angles, ++i) {
            points[i] = Vector2f(ranges[i] * cos(min_angle + (i * angle_inc)), ranges[i] * sin(min_angle + (i * angle_inc)));
        }
    };

    void add_point(Vector2f point) {
        points.append(point);
    }
}