// binomial_distribution
#include <iostream>
#include <random>

using namespace std;

int main()
{
  random_device rnd_seed{};       // use to seed the rng
  mt19937 rnd_engine{rnd_seed()}; // rng

  double p = 0.1; // probability
  bernoulli_distribution dist(p);

  int nTrials = 50;
  int nnz = 0;

  bool outliers[nTrials];

  // generate 5 runs
  for (size_t i = 0; i < nTrials; ++i)
  {
    outliers[i] = dist(rnd_engine);
    cout << outliers[i] << " ";
    (outliers[i] == true) ? nnz++ : nnz;
  }
  cout << endl << "Number of nonzeros = " << nnz << endl;

  return 0;
}