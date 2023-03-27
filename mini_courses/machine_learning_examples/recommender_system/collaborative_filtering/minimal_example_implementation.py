#!/usr/bin/env python3
import os
import sys

import inspect
import pandas as pd
import time

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# currentdir = %pwd
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

# Define the path to the CSV file
csv_path = parentdir+'/movielens_20/rating.csv'
movie_lens_path = parentdir+"/movielens_20"
edited_csv_path = movie_lens_path + "/edited_rating.csv"

if os.path.exists(edited_csv_path):
    print(f'Reading edited csv')
    df = pd.read_csv(edited_csv_path)
    print(f'Successfully read edited csv')
else:
    print(f"creating edited csv")
    # might take a while
    df = pd.read_csv(csv_path)
    df = df.drop("timestamp", axis=1)

    # reassign movie id
    movie_ids = set(df.movieId)
    movie_id_lookup = {}
    for i, movie_id in enumerate(movie_ids):
        movie_id_lookup[movie_id] = i
    # This might take a while
    df["movieId"] = df.apply(lambda row: movie_id_lookup[row.movieId], axis = 1)
    df.userId = df.userId - 1
    df.to_csv(edited_csv_path)
    print(f'Successfully generated edited csv')

small_edited_csv_path = parentdir+"/movielens_20/small_edited_rating.csv"
if os.path.exists(small_edited_csv_path):
    print(f'reading small csv for training and testing')
    small_df = pd.read_csv(small_edited_csv_path)
else:
    print(f'generating small csv for training and testing')
    from collections import Counter
    user_counts = Counter(df.userId)
    movie_counts = Counter(df.movieId)
    common_user_ids = [u for u, c in user_counts.most_common(1000)]
    common_movie_ids = [m for m, c in movie_counts.most_common(200)]

    # Carve the data out
    # df also by default does not create copies. 
    small_df = df[df.userId.isin(common_user_ids) & df.movieId.isin(common_movie_ids)].copy()
    # remap movie and user IDs

    def remap_to_consecutive_numbers(df, cln_name):
        df_cln = df[cln_name]
        unique_cln = set(df_cln)
        lookup = {}
        for i, old_id in enumerate(unique_cln):
            lookup[old_id] = i
        # WEIRD: must use df[cln_name] instead of df_cln
        # This might take a while
        df[cln_name] = df.apply(lambda row: lookup[row[cln_name]], axis = 1)
        return lookup

    remap_to_consecutive_numbers(small_df, "userId")
    remap_to_consecutive_numbers(small_df, "movieId")
    small_df.to_csv(small_edited_csv_path)
print(f'small df loaded')


## Runtime
from sklearn.utils import shuffle
from collections import defaultdict
import threading
import pickle


user2movies_path = parentdir+"/movielens_20/user2movies.pkl"
movies2user_path = parentdir+"/movielens_20/movies2user.pkl"
ratings_path = parentdir+"/movielens_20/ratings.pkl"
train_df_path = parentdir+"/movielens_20/train_df.csv"
test_df_path = parentdir+"/movielens_20/test_df.csv"

DIVIDE_DATA = False
for path in (user2movies_path, movies2user_path, ratings_path, train_df_path, test_df_path):
    if not os.path.exists(path):
        DIVIDE_DATA = True
        break

if DIVIDE_DATA:
    #TODO Remember to remove
    print(f'Dividing and saving data ...')
    small_df = shuffle(small_df)
    TEST_PERCENTAGE = 0.8
    cutoff = int(len(small_df) * TEST_PERCENTAGE)
    train_df = small_df.iloc[:cutoff]
    test_df = small_df.iloc[cutoff:]
    user2movies = defaultdict(lambda: set())
    movies2user = defaultdict(lambda: set())
    ratings = {}
    # use apply, it's faster
    def update_mapping(row):
        user_id = int(row.userId)
        movie_id = int(row.movieId)
        user2movies[user_id].add(movie_id)
        movies2user[movie_id].add(user_id)
        key = (int(row.userId), int(row.movieId))
        ratings[key] = row.rating
    train_df.apply(update_mapping, axis=1)

    with open(user2movies_path, "wb") as f:
        pickle.dump(dict(user2movies), f)
    with open(movies2user_path, "wb") as f:
        pickle.dump(dict(movies2user), f)
    with open(ratings_path, "wb") as f:
        pickle.dump(dict(ratings), f)
    train_df.to_csv(train_df_path)
    test_df.to_csv(test_df_path)
else:
    #TODO Remember to remove
    print(f'Loading training and test data ... ')
    with open(user2movies_path, "rb") as f:
        user2movies = pickle.load(f)
    with open(movies2user_path, "rb") as f:
        movies2user = pickle.load(f)
    with open(ratings_path, "rb") as f:
        ratings = pickle.load(f)
    train_df = pd.read_csv(train_df_path)
    test_df = pd.read_csv(test_df_path)
#TODO Remember to remove
print(f'Training and testing data loaded')

### Runtime 
from collections import namedtuple
import numpy as np
from typing import Set
import heapq
import time

NEIGHBOR_NUM = 25
# threshold on number of movies for neightbors
COMMON_MOVIE_THRE = 5
Weight = namedtuple("Weight",["value", "name"])
# User data are stored in lists, because the indices have been normalized
weight_heapqs = [[Weight(float('-inf'), "") for _ in range(NEIGHBOR_NUM)] for _ in range (len(user2movies))]
means = [float('-inf') for _ in range (len(user2movies))]
stds = [float('-inf') for _ in range (len(user2movies))]
# deviations of each user. A movie deviation is rating - mean
movie_devs = [{} for _ in range (len(user2movies))]
# set that stores frozenset(user_i, user_i')
compared = set()

def save_model():
    training_results_path = os.path.join(movie_lens_path, "training_results.pkl")
    training_results = (weight_heapqs, means, stds, movie_devs, compared)
    with open(training_results_path, "wb") as f:
        for obj in training_results:
            pickle.dump(obj, f)


def get_and_update_mean_std(user: int, movies: Set[int]):
    """Update means and stds, and return them. Nice"""
    if means[user] == float('-inf') or stds[user] != float('-inf'):
        user_all_ratings = np.array([
            ratings[(int(user), int(movie_id))] for movie_id in movies])
        means[user] = np.mean(user_all_ratings)
        # standard deviation of all user ratings
        stds[user] = np.std(user_all_ratings)
    return means[user], stds[user]

def get_and_update_movie_devs(mean: int, user: int, movies: Set[int]):
    """Update movie deviations for a single user. A movie deviation is rating - mean"""
    if not movie_devs[user]:
        user_all_ratings = np.array([
            ratings[(int(user), int(movie_id))] for movie_id in movies])
        dev_ratings = user_all_ratings - mean
        movie_devs[user] = {movie: dev_rating for movie, dev_rating in zip(movies, dev_ratings)}
    return movie_devs[user]

# TODO
debug_count = 0
DEBUG_USER_THRE = 3
start_time = time.perf_counter()
def train():
    # TODO
    global debug_count
    for user, movies in user2movies.items():
        # TODO
        if debug_count % 10 == 0:
            save_model()
            print(f"User count: {debug_count}, time elapsed: {time.perf_counter() - start_time}s")
        debug_count += 1

        mean, std = get_and_update_mean_std(user, movies)
        movie_devs_dict = get_and_update_movie_devs(mean, user, movies)
        for movie_id in movies:
            other_users = movies2user[movie_id]
            for another_user in other_users:
                # See if we have compared
                key_compared = frozenset((user, another_user))
                if key_compared in compared:
                    continue
                compared.add(key_compared)
                another_user_movies = user2movies[another_user]
                common_movies = movies & another_user_movies

                if len(common_movies) < COMMON_MOVIE_THRE:
                    continue

                if another_user == user:
                    continue
                # calculate weight:
                # HACK: we are calculating weights using the stds and movie_devs of all users' movies, instead of the common ones
                # Then, a weight is calculated by multiplying sums of deviations of the common movies
                # That's been tested fine in production. So, this may not work great if the common movie's std deviations are drastically
                # different from the two users over standard deviation
                another_mean, another_std = get_and_update_mean_std(another_user, another_user_movies)
                another_movie_devs_dict = get_and_update_movie_devs(another_mean, another_user, another_user_movies)
                numerator = sum([movie_devs_dict[m] * another_movie_devs_dict[m] for m in common_movies])/len(common_movies)
                denominator = another_std * std

                w_ij = numerator/denominator

                # add to heapque
                heapq.heappushpop(weight_heapqs[user], Weight(w_ij, another_user))
                heapq.heappushpop(weight_heapqs[another_user], Weight(w_ij, user))

        #TODO Remember to remove
        # print(f'weight_heapqs: {weight_heapqs}')

train()


