from Env import Envi
from prio import DQN
import time
import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np

def test_results():
    reward_sum = 0
    for i in range(10):
        observation, target_id = env.reset()
        while True:
            action = RL.choose_action(observation)
            observation_, reward, done, success = env.step(target_id, action)
            reward_sum += reward
            observation = observation_
            if done:
                break
    return reward_sum


def run():
    step = 2919162
    reward_sum = 0
    count_success = 0
    for episode in range(18000, 50000):
        if step > 5000000:
            break
        observation, target_id = env.reset()
        if step > 2919162+200000 and (episode+1)%500 == 0:
            f.write('{reward}, {ep}, {step}, {count}, {time}\n'.format(reward=reward_sum, ep=episode, step=step, count=count_success, time=time.time()-begin))
            f.flush()
            count_success = 0
            reward_sum = 0
            if (episode+1)%1000 == 0:
                RL.saver.save(RL.sess, 'my_net/model.ckpt', global_step=episode+1)
        while True:
            action = RL.choose_action(observation)
            observation_, reward, done, success = env.step(target_id, action)
            if action == 2:
                reward += 0.02
            reward = np.clip(reward, -1, 1)
            reward_sum += reward
            RL.store_transition(observation, action, reward, observation_)
            observation = observation_
            if step > 2919162+200000 and step%5 == 0:
                RL.learn()
            if success:
                count_success += 1
            if done:
                break
            step += 1
            # time.sleep(0.1)

        if step > 2919162+200000 and (episode+1) % 500 == 0:
            print(step, episode+1)

    print('game over')


if __name__ == '__main__':
    f = open('results.txt', 'a')
    begin = time.time()
    f.write(str(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
    env = Envi()
    RL = DQN(5, 8,
                      learning_rate=0.00025,
                      reward_decay=0.95,
                      replace_target_iter=10000,
                      memory_size=200000,
                      e_greedy=1,
                      e_greedy_increment= None,
                      output_graph=False)
    ckpt = tf.train.get_checkpoint_state('my_net/')
    if ckpt and ckpt.model_checkpoint_path:
        RL.saver.restore(RL.sess, ckpt.model_checkpoint_path)
        print('reload success')
    run()
    # RL.plot_cost()