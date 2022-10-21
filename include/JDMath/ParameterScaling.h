///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include <memory.h>
#include <cmath>
#include <cstdint>
#include <limits>

namespace cc
{
   template<typename T>
   static constexpr uint16_t ToCMD16()
   {
      return (T::CMD2 << 8) | T::CMD1;
   }

   template<typename T>
   static constexpr void Initialize(T* result)
   {
      memset(result, 0xFF, sizeof(T));
   }

   template<typename T>
   static constexpr float ScaledNoNaN(const T& native, float resolutionPerBit, float offset = 0.0)
   {
      //TODO account for invalid / error values / not available
      return static_cast<float>(native * resolutionPerBit - offset);
   }

   template<typename T>
   static constexpr T UnscaledNoNaN(float scaled, float resolutionPerBit, float offset = 0.0f)
   {
      //TODO account for invalid / error values / not available
      return static_cast<T>((scaled + offset) / resolutionPerBit + 0.5f);
   }

   template<typename T>
   static constexpr double ScaledNoNaN64(const T& native, const double& resolutionPerBit, const double& offset = 0.0)
   {
      //TODO account for invalid / error values / not available
      return static_cast<double>(native * resolutionPerBit - offset);
   }

   template<typename T>
   static constexpr T UnscaledNoNaN64(const double& scaled, const double& resolutionPerBit, const double& offset = 0.0)
   {
      //TODO account for invalid / error values / not available
      return static_cast<T>((scaled + offset) / resolutionPerBit + 0.5);
   }

   template<typename T>
   static constexpr float Scaled(const T& native, float resolutionPerBit, float offset = 0.0)
   {
      //TODO account for invalid / error values / not available
      return native != std::numeric_limits<T>::max() ?
             static_cast<float>(native * resolutionPerBit - offset) :
             std::numeric_limits<float>::quiet_NaN();
   }

   template<typename T>
   static constexpr T Unscaled(float scaled, float resolutionPerBit, float offset = 0.0f)
   {
      //TODO account for invalid / error values / not available
      return std::isnan(scaled) ?
             std::numeric_limits<T>::max() :
             static_cast<T>((scaled + offset) / resolutionPerBit + 0.5f);
   }

   template<typename T>
   static constexpr double Scaled64(const T& native, const double& resolutionPerBit, const double& offset = 0.0)
   {
      //TODO account for invalid / error values / not available
      return native != std::numeric_limits<T>::max() ?
             static_cast<double>(native * resolutionPerBit - offset) :
             std::numeric_limits<double>::quiet_NaN();
   }

   template<typename T>
   static constexpr T Unscaled64(const double& scaled, const double& resolutionPerBit, const double& offset = 0.0)
   {
      //TODO account for invalid / error values / not available
      return std::isnan(scaled) ?
             std::numeric_limits<T>::max() :
             static_cast<T>((scaled + offset) / resolutionPerBit + 0.5);
   }

}
