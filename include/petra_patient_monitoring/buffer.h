#include <iostream>
#include <vector>
#include <petra_core/tools/Point.h>

//template <class T>
class buffer
{
private:
    int size_;
    std::vector<Point> buffer_;
    std::vector<double> weights_;
    std::vector<double> normalized_weights_;

public:
    buffer(int size) : size_(size)
    {
        std::vector<double> default_weights(size, 1.0 / size);

        weights_ = default_weights;
    }

    buffer(int size, std::vector<double> weights) : size_(size)
    {
        if (weights.size() != (size_t)size)
        {
            std::cout << "Incorrect number of weights for the buffer size, default used." << std::endl;

            std::vector<double> default_weights(size, 1.0 / size);

            weights_ = default_weights;
        }
        else
        {
            weights_ = weights;
        }
    }

    std::vector<double> normalize_weights(size_t size)
    {
        double sum = 0;
        std::vector<double> normalized_weights;

        for (size_t i = 0; i < size; i++)
        {
            sum += weights_.at(i);
        }

        for (size_t i = 0; i < size; i++)
        {
            normalized_weights.push_back(weights_.at(i) / sum);
        }

        return normalized_weights;
    }

    void push(float value)
    {
        push(Point(value, 0.0));
    }

    void push(float x, float y)
    {
        push(Point(x, y));
    }

    void push(Point element)
    {
        if (buffer_.size() == (size_t)size_)
        {
            buffer_.erase(buffer_.begin());
            buffer_.push_back(element);
        }
        else
        {
            buffer_.push_back(element);
            normalized_weights_ = normalize_weights(buffer_.size());
        }
    }

    void push_weight(double weight)
    {
        if (buffer_.size() == 0)
        {
            std::cout << "no value in buffer to apply a weight" << std::endl;
        }
        else
        {
            if (buffer_.size() == (size_t)size_)
            {
                weights_.erase(weights_.begin());
                weights_.push_back(weight);
            }
            else
            {
                weights_.at(buffer_.size() - 1) = weight;
            }

            normalized_weights_ = normalize_weights(buffer_.size());
        }
    }

    // Arithmetic mean of element-wise multiplication of buffer_vector with normalized_weights
    Point get_weighted_sum()
    {
        Point weighted_sum = Point(0, 0);

        for (size_t i = 0; i < buffer_.size(); i++)
        {
            weighted_sum.x += buffer_.at(i).x * normalized_weights_.at(i);
            weighted_sum.y += buffer_.at(i).y * normalized_weights_.at(i);
        }

        return weighted_sum;
    }

    // Geometric mean of UNweighted probability values between 0 to 1
    Point get_geom_mean()
    {
        Point geom_mean = Point(1, 1);

        for (size_t i = 0; i < buffer_.size(); i++)
        {
            geom_mean.x *= buffer_.at(i).x;
            geom_mean.y *= buffer_.at(i).y;
        }

        geom_mean.x = std::pow(geom_mean.x, 1.0 / buffer_.size());
        geom_mean.y = std::pow(geom_mean.y, 1.0 / buffer_.size());

        return geom_mean;
    }

    void print()
    {
        for (size_t i = 0; i < buffer_.size(); i++)
        {
            std::cout << buffer_.at(i).to_string() << " w" << i << ": " << normalized_weights_.at(i) << std::endl;
        }
    }
};